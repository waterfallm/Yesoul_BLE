#include <Arduino.h>
#include <NimBLEDevice.h>

/*
 * Yesoul_BLE
 * -----------
 * Receives data from a Yesoul smart bike (S3, G1M Plus, G1M Max) over BLE and
 * re-broadcasts it under the Cycling Power Service (0x1818) and the Cycling
 * Speed & Cadence Service (0x1816) so that Garmin Edge computers / watches can
 * consume the data.
 *
 * The Yesoul bike exposes its live metrics through the standard Fitness Machine
 * Service (0x1826) on the "Indoor Bike Data" characteristic (0x2AD2).  That
 * characteristic is a sequence of optional fields selected by a 16-bit flags
 * word, so the layout differs between bike models.  We therefore parse the flags
 * and walk the fields in order (the "proper solution" described in the README)
 * instead of relying on hard-coded byte offsets.
 *
 * Supported field encodings (per the FTMS Indoor Bike Data spec):
 *   - Instant Speed        : 2 bytes, x0.01 km/h
 *   - Instant Cadence      : 2 bytes, x0.5 rpm
 *   - Total Distance       : 3 bytes, x0.001 km
 *   - Resistance Level     : 2 bytes
 *   - Instant Power        : 2 bytes, watts
 */

// ---------------------------------------------------------------------------
// Live values received from the bike
// ---------------------------------------------------------------------------
static short powerInstantaneous = 0;    // corrected instantaneous power (watts)
static short cadenceInstantaneous = 0;  // rpm
static float speedKmh = 0.0f;           // km/h
static short resistance = 0;            // resistance level (FTMS sint16)

// ---------------------------------------------------------------------------
// Device profile: lets us apply per-model corrections based on the advertised
// BLE name of the bike we connect to.
// ---------------------------------------------------------------------------
enum BikeModel
{
  BIKE_UNKNOWN = 0,
  BIKE_S3,       // Yesoul S3        (advertised name "YS_G1_...")
  BIKE_G1MPLUS,  // Yesoul G1M Plus  (advertised name "YS_G1MPLUS...")
  BIKE_G1MMAX    // Yesoul G1M Max   (advertised name "YS_G1MMAX...")
};

static BikeModel bikeModel = BIKE_UNKNOWN;
static float powerScale = 1.0f; // incoming power is multiplied by this value
static String bikeName;         // advertised name of the connected bike

// ---------------------------------------------------------------------------
// Client side: connect to the bike's Fitness Machine Service
// ---------------------------------------------------------------------------
static BLEUUID serviceUUID("1826"); // Fitness Machine Service
static BLEUUID charUUID("2ad2");    // Indoor Bike Data characteristic

static boolean doConnect = false;
static boolean connected = false;
static boolean doScan = false;
static BLERemoteCharacteristic *pRemoteCharacteristic;
// NimBLE 2.x deletes the NimBLEAdvertisedDevice objects owned by the scan
// results whenever the scan is restarted/cleared, so holding a raw pointer to
// the discovered device (as 1.x code does) leaves a dangling pointer.  Store a
// copy of its address and name instead and connect by address.
static NimBLEAddress myDeviceAddress;
static std::string myDeviceName;

// ---------------------------------------------------------------------------
// Server side: re-broadcast as Cycling Power + Cycling Speed & Cadence
// ---------------------------------------------------------------------------
static NimBLEServer *pServer;
NimBLECharacteristic *CyclingPowerFeature = NULL;
NimBLECharacteristic *CyclingPowerMeasurement = NULL;
NimBLECharacteristic *CyclingPowerSensorLocation = NULL;
NimBLECharacteristic *SpeedCadenceMeasurement = NULL;

unsigned char bleBuffer[8];
unsigned char slBuffer[1];
unsigned char fBuffer[4];
// CSC Measurement (0x2A5B) with both flag bits set:
//   flags(1) + cumulative wheel revolutions(4) + last wheel event time(2)
//            + cumulative crank revolutions(2) + last crank event time(2) = 11
unsigned char scBuffer[11];

unsigned short revolutions = 0;     // accumulated crank revolutions (Cycling Power)
unsigned short timestamp = 0;        // Cycling Power timestamp (1/1024 s)
unsigned short flags = 0x20;         // CPS flags bit5 = Crank Revolution Data Present
byte sensorlocation = 0x0D;          // 0x0D = Crank

// Cumulative wheel revolutions are a uint32 per the CSC spec (a uint16 would
// wrap after ~164 km at a 2.5 m wheel circumference).
unsigned long wheelRevolutions = 0;  // accumulated wheel revolutions (CSC)
unsigned short wheelTimestamp = 0;   // CSC last wheel event time (1/1024 s)
unsigned short crankTimestamp = 0;   // CSC last crank event time (1/1024 s)

long lastNotify = 0;
long lastRevolution = 0;
long lastWheelRevolution = 0;

// Both CPS and CSC event-time fields are a free-running 1/1024 s clock.  The
// conversion is done in 64-bit arithmetic because "millis() * 1024" overflows a
// 32-bit unsigned long after only ~69.9 minutes, which would corrupt the
// timestamps (and therefore the speed/cadence deltas) on longer rides.
static inline unsigned short bleEventTime()
{
  return (unsigned short)((((uint64_t)millis() * 1024ULL) / 1000ULL) & 0xFFFFULL);
}

// ---------------------------------------------------------------------------
// Server callbacks
// ---------------------------------------------------------------------------
// NimBLE-Arduino 2.x replaced the ble_gap_conn_desc* parameter of every
// connection-oriented callback with a NimBLEConnInfo& reference.  Overriding
// the old 1.x signatures would still *compile* (C++ does not require an
// override to match a base-class virtual) but the methods would silently never
// be called, so every callback below is written against the 2.x signatures.
class ServerCallbacks : public NimBLEServerCallbacks
{
  void onConnect(NimBLEServer *pServer, NimBLEConnInfo &connInfo)
  {
    Serial.println("Client connected");
    Serial.print("Client address: ");
    Serial.println(connInfo.getAddress().toString().c_str());
    // Request a tighter connection interval for lower latency.
    pServer->updateConnParams(connInfo.getConnHandle(), 24, 48, 0, 60);
  };
  void onDisconnect(NimBLEServer *pServer, NimBLEConnInfo &connInfo, int reason)
  {
    Serial.print("Client disconnected - start advertising (reason ");
    Serial.print(reason);
    Serial.println(")");
    NimBLEDevice::startAdvertising();
  };
  void onMTUChange(uint16_t MTU, NimBLEConnInfo &connInfo)
  {
    Serial.printf("MTU updated: %u for connection ID: %u\n", MTU, connInfo.getConnHandle());
  };
};

class CharacteristicCallbacks : public NimBLECharacteristicCallbacks
{
  // NimBLE 2.x returns a NimBLEAttValue from getValue(); convert it explicitly
  // to std::string instead of relying on the 1.x std::string return type.
  void onRead(NimBLECharacteristic *pCharacteristic, NimBLEConnInfo &connInfo)
  {
    Serial.print(pCharacteristic->getUUID().toString().c_str());
    Serial.print(": onRead(), value: ");
    Serial.println(std::string(pCharacteristic->getValue()).c_str());
  };
  void onWrite(NimBLECharacteristic *pCharacteristic, NimBLEConnInfo &connInfo)
  {
    Serial.print(pCharacteristic->getUUID().toString().c_str());
    Serial.print(": onWrite(), value: ");
    Serial.println(std::string(pCharacteristic->getValue()).c_str());
  };
  // onNotify() was removed in 2.x, and onStatus() now takes a plain int code
  // instead of the deleted Status enum.
  void onStatus(NimBLECharacteristic *pCharacteristic, NimBLEConnInfo &connInfo, int code)
  {
    Serial.print("Notification/Indication status, return code: ");
    Serial.print(code);
    Serial.print(", ");
    Serial.println(NimBLEUtils::returnCodeToString(code));
  };
  void onSubscribe(NimBLECharacteristic *pCharacteristic, NimBLEConnInfo &connInfo, uint16_t subValue)
  {
    String str = "Client ID: ";
    str += connInfo.getConnHandle();
    str += " Address: ";
    str += connInfo.getAddress().toString().c_str();
    if (subValue == 0)
      str += " Unsubscribed to ";
    else if (subValue == 1)
      str += " Subscribed to notifications for ";
    else if (subValue == 2)
      str += " Subscribed to indications for ";
    else if (subValue == 3)
      str += " Subscribed to notifications and indications for ";
    str += std::string(pCharacteristic->getUUID()).c_str();
    Serial.println(str);
  };
};

class DescriptorCallbacks : public NimBLEDescriptorCallbacks
{
  void onWrite(NimBLEDescriptor *pDescriptor, NimBLEConnInfo &connInfo)
  {
    std::string dscVal = pDescriptor->getValue();
    Serial.print("Descriptor written value:");
    Serial.println(dscVal.c_str());
  };
  void onRead(NimBLEDescriptor *pDescriptor, NimBLEConnInfo &connInfo)
  {
    Serial.print(pDescriptor->getUUID().toString().c_str());
    Serial.println(" Descriptor read");
  };
};

// ---------------------------------------------------------------------------
// Device profile selection based on the advertised name of the bike.
// Yesoul advertises:
//   S3        -> "YS_G1_..."
//   G1M Plus  -> "YS_G1MPLUS..."
//   G1M Max   -> "YS_G1MMAX..."
// ---------------------------------------------------------------------------
static void applyBikeProfile(const String &name)
{
  bikeName = name;
  String upper = name;
  upper.toUpperCase();

  if (upper.startsWith("YS_G1MPLUS"))
  {
    bikeModel = BIKE_G1MPLUS;
    powerScale = 1.0f; // G1M Plus reports power directly, no S3 correction needed
  }
  else if (upper.startsWith("YS_G1MMAX"))
  {
    bikeModel = BIKE_G1MMAX;
    powerScale = 1.0f; // G1M Max shares the G1M data format
  }
  else if (upper.startsWith("YS_G1_"))
  {
    bikeModel = BIKE_S3;
    powerScale = 1.28f; // legacy S3 correction (kept for backwards compatibility)
  }
  else
  {
    bikeModel = BIKE_UNKNOWN;
    powerScale = 1.0f;
  }

  const char *modelName = "Unknown";
  switch (bikeModel)
  {
  case BIKE_UNKNOWN:
    modelName = "Unknown";
    break;
  case BIKE_S3:
    modelName = "Yesoul S3";
    break;
  case BIKE_G1MPLUS:
    modelName = "Yesoul G1M Plus";
    break;
  case BIKE_G1MMAX:
    modelName = "Yesoul G1M Max";
    break;
  }
  Serial.printf("Detected bike: %s (name=%s) powerScale=%.3f\n", modelName, name.c_str(), powerScale);
}

// ---------------------------------------------------------------------------
// Parse the FTMS Indoor Bike Data (0x2AD2) notification.
// Reads the 16-bit flags word and walks the present fields in order so that it
// works for any Yesoul model regardless of which fields it chooses to send.
// ---------------------------------------------------------------------------
static void parseIndoorBikeData(uint8_t *pData, size_t length)
{
  if (length < 2)
    return;

  uint16_t flagsWord = ((uint16_t)pData[1] << 8) | pData[0];

  // FTMS Indoor Bike Data flag bits
  const uint16_t F_MORE_DATA       = 0x0001;
  const uint16_t F_AVG_SPEED       = 0x0002;
  const uint16_t F_INST_CADENCE    = 0x0004;
  const uint16_t F_AVG_CADENCE     = 0x0008;
  const uint16_t F_TOT_DISTANCE    = 0x0010;
  const uint16_t F_RESISTANCE_LVL  = 0x0020;
  const uint16_t F_INST_POWER      = 0x0040;
  const uint16_t F_AVG_POWER       = 0x0080;
  const uint16_t F_EXP_ENERGY      = 0x0100;
  const uint16_t F_HEART_RATE      = 0x0200;
  const uint16_t F_METABOLIC       = 0x0400;
  const uint16_t F_ELAPSED_TIME    = 0x0800;
  const uint16_t F_REMAINING_TIME  = 0x1000;

  size_t idx = 2; // fields begin after the 2-byte flags word

  // Track which fields the bike actually sent.  Falling back to an *average*
  // value whenever the instantaneous value happens to be zero is wrong: while
  // coasting the bike legitimately reports 0 W / 0 rpm / 0 km/h, and
  // substituting the session average makes the re-broadcast power read
  // roughly 30-50 W too high (the offset reported in issue #2).  Averages are
  // therefore only used when the corresponding instantaneous field is absent.
  bool haveSpeed = false;
  bool haveCadence = false;
  bool havePower = false;

  auto readU16 = [&]() -> uint16_t {
    if (length < idx + 2)
      return 0;
    uint16_t v = ((uint16_t)pData[idx + 1] << 8) | pData[idx];
    idx += 2;
    return v;
  };
  auto skip = [&](size_t n) { idx += n; };

  // Instant Speed (x0.01 km/h). Present when the "more data" bit is NOT set.
  if ((flagsWord & F_MORE_DATA) == 0)
  {
    uint16_t raw = readU16();
    speedKmh = raw / 100.0f;
    haveSpeed = true;
  }

  // Average Speed (only if no instant speed was provided)
  if (flagsWord & F_AVG_SPEED)
  {
    uint16_t raw = readU16();
    if (!haveSpeed)
    {
      speedKmh = raw / 100.0f;
      haveSpeed = true;
    }
  }

  // Instant Cadence (x0.5 rpm)
  if (flagsWord & F_INST_CADENCE)
  {
    uint16_t raw = readU16();
    cadenceInstantaneous = raw / 2;
    haveCadence = true;
  }

  // Average Cadence (only if no instant cadence was provided)
  if (flagsWord & F_AVG_CADENCE)
  {
    uint16_t raw = readU16();
    if (!haveCadence)
    {
      cadenceInstantaneous = raw / 2;
      haveCadence = true;
    }
  }

  // Total Distance (3 bytes, x0.001 km) - not re-broadcast, skip it
  if (flagsWord & F_TOT_DISTANCE)
    skip(3);

  // Resistance Level (sint16).  Only used for logging, but read it with the
  // correct signedness instead of truncating the high byte away.
  if (flagsWord & F_RESISTANCE_LVL)
  {
    resistance = (short)readU16();
  }

  // Instant Power (watts)
  short powerRaw = 0;
  if (flagsWord & F_INST_POWER)
  {
    powerRaw = (short)readU16();
    havePower = true;
  }

  // Average Power (only if the bike did not send an instantaneous power field)
  if (flagsWord & F_AVG_POWER)
  {
    short raw = (short)readU16();
    if (!havePower)
    {
      powerRaw = raw;
      havePower = true;
    }
  }

  // Apply the per-model power correction and clamp to a sane range.
  float corrected = powerRaw * powerScale;
  if (corrected < 0)
    corrected = 0;
  if (corrected > 2000)
    corrected = 2000;
  powerInstantaneous = (short)corrected;

  // Remaining optional fields are not needed, but consume them with their
  // correct spec sizes so idx stays sane if anything is added after them.
  if (flagsWord & F_EXP_ENERGY)
    skip(5); // Total Energy(2) + Energy Per Hour(2) + Energy Per Minute(1)
  if (flagsWord & F_HEART_RATE)
    skip(1); // uint8
  if (flagsWord & F_METABOLIC)
    skip(1); // Metabolic Equivalent is a uint8
  if (flagsWord & F_ELAPSED_TIME)
    skip(2); // uint16 (was incorrectly skipped as 3 bytes)
  if (flagsWord & F_REMAINING_TIME)
    skip(2); // uint16 (was incorrectly skipped as 3 bytes)

  Serial.printf("Power = %d | Cadence = %d | Speed = %.2f km/h | Resistance = %d\n",
                powerInstantaneous, cadenceInstantaneous, speedKmh, resistance);
}

// ---------------------------------------------------------------------------
// Client callback: invoked for each 0x2AD2 notification from the bike
// ---------------------------------------------------------------------------
static void notifyCallback(
    BLERemoteCharacteristic *pBLERemoteCharacteristic,
    uint8_t *pData,
    size_t length,
    bool isNotify)
{
  parseIndoorBikeData(pData, length);
}

class MyClientCallback : public BLEClientCallbacks
{
  void onConnect(BLEClient *pclient)
  {
  }
  // 2.x appends a disconnect reason code to this callback.
  void onDisconnect(BLEClient *pclient, int reason)
  {
    connected = false;
    Serial.print("onDisconnect, reason: ");
    Serial.println(reason);
  }
};

bool connectToServer()
{
  Serial.print("Forming a connection to ");
  Serial.println(myDeviceAddress.toString().c_str());

  BLEClient *pClient = BLEDevice::createClient();
  Serial.println(" - Created client");

  pClient->setClientCallbacks(new MyClientCallback());

  pClient->connect(myDeviceAddress);
  Serial.println(" - Connected to server");

  // Apply the per-model profile (power scale, etc.) now that we know the name.
  applyBikeProfile(String(myDeviceName.c_str()));

  BLERemoteService *pRemoteService = pClient->getService(serviceUUID);
  if (pRemoteService == nullptr)
  {
    Serial.print("Failed to find our service UUID: ");
    Serial.println(serviceUUID.toString().c_str());
    pClient->disconnect();
    return false;
  }
  Serial.println(" - Found our service");

  pRemoteCharacteristic = pRemoteService->getCharacteristic(charUUID);
  if (pRemoteCharacteristic == nullptr)
  {
    Serial.print("Failed to find our characteristic UUID: ");
    Serial.println(charUUID.toString().c_str());
    pClient->disconnect();
    return false;
  }
  Serial.println(" - Found our characteristic");

  if (pRemoteCharacteristic->canRead())
  {
    // readValue() returns a NimBLEAttValue in 2.x (it returned std::string in
    // 1.x), so convert it explicitly.
    std::string value = pRemoteCharacteristic->readValue();
    Serial.print("The characteristic value was: ");
    Serial.println(value.c_str());
  }

  // subscribe() is the non-deprecated replacement for registerForNotify() in
  // NimBLE-Arduino; it takes the same notify_callback signature.
  if (pRemoteCharacteristic->canNotify())
    pRemoteCharacteristic->subscribe(true, notifyCallback);

  connected = true;
  return true;
}

// ---------------------------------------------------------------------------
// Scan for the bike advertising the Fitness Machine Service
// ---------------------------------------------------------------------------
class MyAdvertisedDeviceCallbacks : public BLEAdvertisedDeviceCallbacks
{
  void onResult(const BLEAdvertisedDevice *advertisedDevice)
  {
    Serial.print("BLE Advertised Device found: ");
    Serial.println(advertisedDevice->toString().c_str());

    if (advertisedDevice->haveServiceUUID() && advertisedDevice->isAdvertisingService(serviceUUID))
    {
      BLEDevice::getScan()->stop();
      // Copy the address/name rather than keeping the pointer: 2.x owns and
      // deletes these objects when the scan results are cleared.
      myDeviceAddress = advertisedDevice->getAddress();
      myDeviceName = advertisedDevice->getName();
      doConnect = true;
      doScan = true;
    }
  }
};

// ---------------------------------------------------------------------------
// Setup: create both re-broadcast services and start scanning
// ---------------------------------------------------------------------------
void setup()
{
  Serial.begin(115200);
  Serial.println("Starting NimBLE Server");

  NimBLEDevice::init("Yesoul_CP");
  NimBLEDevice::setPower(9); // +9 dBm (2.x takes a plain int, not esp_power_level_t)

  pServer = NimBLEDevice::createServer();
  pServer->setCallbacks(new ServerCallbacks());

  // --- Cycling Power Service (0x1818) ---
  // Cycling Power Feature (0x2A65) is a 32-bit little-endian bitfield:
  //   bit2 = Wheel Revolution Data Supported
  //   bit3 = Crank Revolution Data Supported
  // We send crank revolution data, so the value must be 0x00000008, encoded
  // least-significant octet first.  (The previous value put 0x08 in byte 3,
  // which set bit27 - a Reserved For Future Use bit - and declared no crank
  // support even though the measurement advertises crank data.)
  fBuffer[0] = 0x08;
  fBuffer[1] = 0x00;
  fBuffer[2] = 0x00;
  fBuffer[3] = 0x00;
  slBuffer[0] = sensorlocation & 0xff;

  NimBLEService *pPowerService = pServer->createService("1818");
  CyclingPowerFeature = pPowerService->createCharacteristic(
      "2A65", NIMBLE_PROPERTY::READ);
  CyclingPowerSensorLocation = pPowerService->createCharacteristic(
      "2A5D", NIMBLE_PROPERTY::READ);
  CyclingPowerMeasurement = pPowerService->createCharacteristic(
      "2A63", NIMBLE_PROPERTY::READ | NIMBLE_PROPERTY::NOTIFY);

  CyclingPowerFeature->setValue(fBuffer, sizeof(fBuffer));
  CyclingPowerSensorLocation->setValue(slBuffer, sizeof(slBuffer));
  // Seed the measurement with its correct 8-byte layout rather than the
  // 1-byte sensor-location buffer.
  CyclingPowerMeasurement->setValue(bleBuffer, sizeof(bleBuffer));
  // NimBLEService::start() is a no-op in 2.x; services start with the server.

  // --- Cycling Speed & Cadence Service (0x1816) ---
  // Lets Garmin derive distance from the reported wheel revolutions.
  // 0x1816 is the SIG-assigned CSC service; 0x181C is the *User Data* service,
  // so advertising the CSC characteristics under 0x181C meant no client could
  // ever discover them (the reported "missing distance" symptom).
  NimBLEService *pScsService = pServer->createService("1816");
  // CSC Measurement is 0x2A5B and CSC Feature is 0x2A5C (these were swapped).
  SpeedCadenceMeasurement = pScsService->createCharacteristic(
      "2A5B", NIMBLE_PROPERTY::READ | NIMBLE_PROPERTY::NOTIFY); // CSC Measurement
  NimBLECharacteristic *scsFeature = pScsService->createCharacteristic(
      "2A5C", NIMBLE_PROPERTY::READ); // CSC Feature

  // CSC Feature is a 16-bit bitfield: bit0 = Wheel Revolution Data Supported,
  // bit1 = Crank Revolution Data Supported.
  unsigned char scsFeatureBuf[2] = {0x03, 0x00};
  scsFeature->setValue(scsFeatureBuf, 2);
  // Seed with the flags byte only (0x00 = no revolution data present yet) so
  // the initial value is self-consistent; a full 11-byte buffer would advertise
  // no fields while carrying trailing zero bytes.
  SpeedCadenceMeasurement->setValue(scBuffer, 1);

  NimBLEAdvertising *pAdvertising = NimBLEDevice::getAdvertising();
  pAdvertising->addServiceUUID(pPowerService->getUUID());
  pAdvertising->addServiceUUID(pScsService->getUUID());
  // 2.x no longer includes the device name or TX power in the advertisement by
  // default, so set the name explicitly or clients show an unnamed device.
  pAdvertising->setName("Yesoul_CP");
  pAdvertising->enableScanResponse(true);
  pAdvertising->start();

  Serial.println("Advertising Started");

  Serial.println("Starting Arduino BLE Client application...");

  BLEScan *pBLEScan = BLEDevice::getScan();
  pBLEScan->setScanCallbacks(new MyAdvertisedDeviceCallbacks());
  pBLEScan->setInterval(1349);
  pBLEScan->setWindow(449);
  pBLEScan->setActiveScan(true);
  // 2.x takes all durations in milliseconds, not seconds.
  pBLEScan->start(5000, false);
}

// ---------------------------------------------------------------------------
// Loop: accumulate revolutions/timestamps and re-broadcast each second
// ---------------------------------------------------------------------------
void loop()
{
  if (doConnect == true)
  {
    if (connectToServer())
      Serial.println("We are now connected to the BLE Server.");
    else
      Serial.println("We have failed to connect to the server; there is nothing more we will do.");
    doConnect = false;
  }

  if (connected)
  {
    // data handled via notifyCallback
  }
  else if (doScan)
  {
    BLEDevice::getScan()->start(0); // resume scanning after a disconnect
  }

  // --- Accumulate crank revolutions from cadence (Cycling Power / CSC) ---
  if (cadenceInstantaneous != 0 && (millis()) >= (lastRevolution + (60000 / cadenceInstantaneous)))
  {
    revolutions++;
    timestamp = bleEventTime();
    crankTimestamp = timestamp;
    lastRevolution = millis();
  }

  // --- Accumulate wheel revolutions from speed (CSC) ---
  // Approximate the wheel circumference for a spin bike as ~2.5 m so that the
  // reported speed stays consistent with the derived distance.
  static const float wheelCircumferenceM = 2.5f;
  if (speedKmh > 0.0f)
  {
    float metersPerSec = speedKmh / 3.6f;
    float revsPerSec = metersPerSec / wheelCircumferenceM;
    unsigned long intervalMs = (revsPerSec > 0) ? (unsigned long)(1000.0f / revsPerSec) : 0;
    if (intervalMs > 0 && (millis()) >= (lastWheelRevolution + intervalMs))
    {
      wheelRevolutions++;
      wheelTimestamp = bleEventTime();
      lastWheelRevolution = millis();
    }
  }

  // --- Re-broadcast every second while a client is connected ---
  if (millis() - lastNotify >= 1000)
  {
    if (pServer->getConnectedCount() > 0)
    {
      // Cycling Power Measurement (0x2A63).
      // flags(2) + instantaneous power sint16(2)
      //        + cumulative crank revolutions uint16(2)
      //        + last crank event time uint16(2) = 8 bytes.
      bleBuffer[0] = flags & 0xff;
      bleBuffer[1] = (flags >> 8) & 0xff;
      bleBuffer[2] = powerInstantaneous & 0xff;
      bleBuffer[3] = (powerInstantaneous >> 8) & 0xff;
      bleBuffer[4] = revolutions & 0xff;
      bleBuffer[5] = (revolutions >> 8) & 0xff;
      bleBuffer[6] = timestamp & 0xff;
      bleBuffer[7] = (timestamp >> 8) & 0xff;
      CyclingPowerMeasurement->setValue(bleBuffer, sizeof(bleBuffer));
      CyclingPowerMeasurement->notify();

      // CSC Measurement (0x2A5B).  The flags field is a single byte:
      //   bit0 = Wheel Revolution Data Present
      //   bit1 = Crank Revolution Data Present
      // There is no "instantaneous cadence" field - clients derive cadence from
      // the cumulative crank revolution counter and the last crank event time.
      // The previous layout used a 2-byte flags word and packed the wheel
      // revolutions as a uint16, so every field was misaligned and clients
      // discarded the values (this is why distance never appeared).
      unsigned char scsFlags = 0;
      if (speedKmh > 0.0f)
        scsFlags |= 0x01; // wheel revolution data present
      if (cadenceInstantaneous > 0)
        scsFlags |= 0x02; // crank revolution data present

      size_t o = 0;
      scBuffer[o++] = scsFlags;
      if (scsFlags & 0x01)
      {
        // Cumulative Wheel Revolutions (uint32, little-endian)
        scBuffer[o++] = (unsigned char)(wheelRevolutions & 0xff);
        scBuffer[o++] = (unsigned char)((wheelRevolutions >> 8) & 0xff);
        scBuffer[o++] = (unsigned char)((wheelRevolutions >> 16) & 0xff);
        scBuffer[o++] = (unsigned char)((wheelRevolutions >> 24) & 0xff);
        // Last Wheel Event Time (uint16, 1/1024 s)
        scBuffer[o++] = (unsigned char)(wheelTimestamp & 0xff);
        scBuffer[o++] = (unsigned char)((wheelTimestamp >> 8) & 0xff);
      }
      if (scsFlags & 0x02)
      {
        // Cumulative Crank Revolutions (uint16)
        scBuffer[o++] = (unsigned char)(revolutions & 0xff);
        scBuffer[o++] = (unsigned char)((revolutions >> 8) & 0xff);
        // Last Crank Event Time (uint16, 1/1024 s)
        scBuffer[o++] = (unsigned char)(crankTimestamp & 0xff);
        scBuffer[o++] = (unsigned char)((crankTimestamp >> 8) & 0xff);
      }
      SpeedCadenceMeasurement->setValue(scBuffer, o);
      SpeedCadenceMeasurement->notify();

      lastNotify = millis();
    }
  }

  if (pServer->getConnectedCount() == 0)
  {
    powerInstantaneous = 0;
  }
}
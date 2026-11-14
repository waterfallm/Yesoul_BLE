#include <Arduino.h>
#include <NimBLEDevice.h>

/*
 * Yesoul_BLE
 * -----------
 * Receives data from a Yesoul smart bike (S3, G1M Plus, G1M Max) over BLE and
 * re-broadcasts it under the Cycling Power Service (0x1818) and the Cycling
 * Speed & Cadence Service (0x181C) so that Garmin Edge computers / watches can
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
static uint8_t resistance = 0;          // raw resistance level

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
static BLEAdvertisedDevice *myDevice;

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
unsigned char scBuffer[8]; // CSC measurement: flags(2) + cadence(2) + wheelRev(2) + wheelTime(2)

unsigned short revolutions = 0;     // accumulated crank revolutions (Cycling Power)
unsigned short timestamp = 0;        // Cycling Power timestamp (1/1024 s)
unsigned short flags = 0x20;         // Cycling Power flags: bit5 = crank revolution present
byte sensorlocation = 0x0D;          // 0x0D = Crank

unsigned short wheelRevolutions = 0; // accumulated wheel revolutions (CSC)
unsigned short wheelTimestamp = 0;   // CSC timestamp (1/1024 s)

long lastNotify = 0;
long lastRevolution = 0;
long lastWheelRevolution = 0;

// ---------------------------------------------------------------------------
// Server callbacks
// ---------------------------------------------------------------------------
class ServerCallbacks : public NimBLEServerCallbacks
{
  void onConnect(NimBLEServer *pServer)
  {
    Serial.println("Client connected");
    NimBLEDevice::startAdvertising();
  };
  void onConnect(NimBLEServer *pServer, ble_gap_conn_desc *desc)
  {
    Serial.print("Client address: ");
    Serial.println(NimBLEAddress(desc->peer_ota_addr).toString().c_str());
    // Request a tighter connection interval for lower latency.
    pServer->updateConnParams(desc->conn_handle, 24, 48, 0, 60);
  };
  void onDisconnect(NimBLEServer *pServer)
  {
    Serial.println("Client disconnected - start advertising");
    NimBLEDevice::startAdvertising();
  };
  void onMTUChange(uint16_t MTU, ble_gap_conn_desc *desc)
  {
    Serial.printf("MTU updated: %u for connection ID: %u\n", MTU, desc->conn_handle);
  };
};

class CharacteristicCallbacks : public NimBLECharacteristicCallbacks
{
  void onRead(NimBLECharacteristic *pCharacteristic)
  {
    Serial.print(pCharacteristic->getUUID().toString().c_str());
    Serial.print(": onRead(), value: ");
    Serial.println(pCharacteristic->getValue().c_str());
  };
  void onWrite(NimBLECharacteristic *pCharacteristic)
  {
    Serial.print(pCharacteristic->getUUID().toString().c_str());
    Serial.print(": onWrite(), value: ");
    Serial.println(pCharacteristic->getValue().c_str());
  };
  void onNotify(NimBLECharacteristic *pCharacteristic)
  {
    Serial.println("Sending notification to clients");
  };
  void onStatus(NimBLECharacteristic *pCharacteristic, Status status, int code)
  {
    String str = ("Notification/Indication status code: ");
    str += status;
    str += ", return code: ";
    str += code;
    str += ", ";
    str += NimBLEUtils::returnCodeToString(code);
    Serial.println(str);
  };
  void onSubscribe(NimBLECharacteristic *pCharacteristic, ble_gap_conn_desc *desc, uint16_t subValue)
  {
    String str = "Client ID: ";
    str += desc->conn_handle;
    str += " Address: ";
    str += std::string(NimBLEAddress(desc->peer_ota_addr)).c_str();
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
  void onWrite(NimBLEDescriptor *pDescriptor)
  {
    std::string dscVal = pDescriptor->getValue();
    Serial.print("Descriptor written value:");
    Serial.println(dscVal.c_str());
  };
  void onRead(NimBLEDescriptor *pDescriptor)
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
  }

  // Average Speed
  if (flagsWord & F_AVG_SPEED)
  {
    uint16_t raw = readU16();
    if (speedKmh == 0.0f)
      speedKmh = raw / 100.0f; // fall back to average if no instant speed
  }

  // Instant Cadence (x0.5 rpm)
  if (flagsWord & F_INST_CADENCE)
  {
    uint16_t raw = readU16();
    cadenceInstantaneous = raw / 2;
  }

  // Average Cadence
  if (flagsWord & F_AVG_CADENCE)
  {
    uint16_t raw = readU16();
    if (cadenceInstantaneous == 0)
      cadenceInstantaneous = raw / 2;
  }

  // Total Distance (3 bytes, x0.001 km) - not re-broadcast, skip it
  if (flagsWord & F_TOT_DISTANCE)
    skip(3);

  // Resistance Level
  if (flagsWord & F_RESISTANCE_LVL)
  {
    uint16_t raw = readU16();
    resistance = raw & 0xFF;
  }

  // Instant Power (watts)
  short powerRaw = 0;
  if (flagsWord & F_INST_POWER)
  {
    powerRaw = (short)readU16();
  }

  // Average Power (fallback if instant power is absent/zero)
  if (flagsWord & F_AVG_POWER)
  {
    uint16_t raw = readU16();
    if (powerRaw == 0)
      powerRaw = (short)raw;
  }

  // Apply the per-model power correction and clamp to a sane range.
  float corrected = powerRaw * powerScale;
  if (corrected < 0)
    corrected = 0;
  if (corrected > 2000)
    corrected = 2000;
  powerInstantaneous = (short)corrected;

  // Remaining optional fields are not needed, but consume them to keep idx sane.
  if (flagsWord & F_EXP_ENERGY)
    skip(5); // energy total(2) + per hour(2) + per minute(1)
  if (flagsWord & F_HEART_RATE)
    skip(1);
  if (flagsWord & F_METABOLIC)
    skip(0);
  if (flagsWord & F_ELAPSED_TIME)
    skip(3);
  if (flagsWord & F_REMAINING_TIME)
    skip(3);

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
  void onDisconnect(BLEClient *pclient)
  {
    connected = false;
    Serial.println("onDisconnect");
  }
};

bool connectToServer()
{
  Serial.print("Forming a connection to ");
  Serial.println(myDevice->getAddress().toString().c_str());

  BLEClient *pClient = BLEDevice::createClient();
  Serial.println(" - Created client");

  pClient->setClientCallbacks(new MyClientCallback());

  pClient->connect(myDevice);
  Serial.println(" - Connected to server");

  // Apply the per-model profile (power scale, etc.) now that we know the name.
  applyBikeProfile(myDevice->getName());

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
    std::string value = pRemoteCharacteristic->readValue();
    Serial.print("The characteristic value was: ");
    Serial.println(value.c_str());
  }

  if (pRemoteCharacteristic->canNotify())
    pRemoteCharacteristic->registerForNotify(notifyCallback);

  connected = true;
  return true;
}

// ---------------------------------------------------------------------------
// Scan for the bike advertising the Fitness Machine Service
// ---------------------------------------------------------------------------
class MyAdvertisedDeviceCallbacks : public BLEAdvertisedDeviceCallbacks
{
  void onResult(BLEAdvertisedDevice *advertisedDevice)
  {
    Serial.print("BLE Advertised Device found: ");
    Serial.println(advertisedDevice->toString().c_str());

    if (advertisedDevice->haveServiceUUID() && advertisedDevice->isAdvertisingService(serviceUUID))
    {
      BLEDevice::getScan()->stop();
      myDevice = advertisedDevice; // just save the reference
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
  NimBLEDevice::setPower(ESP_PWR_LVL_P9); // +9db

  pServer = NimBLEDevice::createServer();
  pServer->setCallbacks(new ServerCallbacks());

  // --- Cycling Power Service (0x1818) ---
  fBuffer[0] = 0x00;
  fBuffer[1] = 0x00;
  fBuffer[2] = 0x00;
  fBuffer[3] = 0x08; // feature: left+right crank, accumulated power, etc.
  slBuffer[0] = sensorlocation & 0xff;

  NimBLEService *pPowerService = pServer->createService("1818");
  CyclingPowerFeature = pPowerService->createCharacteristic(
      "2A65", NIMBLE_PROPERTY::READ);
  CyclingPowerSensorLocation = pPowerService->createCharacteristic(
      "2A5D", NIMBLE_PROPERTY::READ);
  CyclingPowerMeasurement = pPowerService->createCharacteristic(
      "2A63", NIMBLE_PROPERTY::READ | NIMBLE_PROPERTY::NOTIFY);

  CyclingPowerFeature->setValue(fBuffer, 4);
  CyclingPowerSensorLocation->setValue(slBuffer, 1);
  CyclingPowerMeasurement->setValue(slBuffer, 1);
  pPowerService->start();

  // --- Cycling Speed & Cadence Service (0x181C) ---
  // Lets Garmin derive distance from the reported speed.
  NimBLEService *pScsService = pServer->createService("181C");
  NimBLECharacteristic *scsFeature = pScsService->createCharacteristic(
      "2A5B", NIMBLE_PROPERTY::READ); // Speed/Cadence Feature
  SpeedCadenceMeasurement = pScsService->createCharacteristic(
      "2A5C", NIMBLE_PROPERTY::READ | NIMBLE_PROPERTY::NOTIFY); // Speed/Cadence Measurement

  // CSC feature: bit0 = wheel revolution supported, bit1 = cadence supported.
  unsigned char scsFeatureBuf[2] = {0x03, 0x00};
  scsFeature->setValue(scsFeatureBuf, 2);
  SpeedCadenceMeasurement->setValue(scBuffer, 8);
  pScsService->start();

  NimBLEAdvertising *pAdvertising = NimBLEDevice::getAdvertising();
  pAdvertising->addServiceUUID(pPowerService->getUUID());
  pAdvertising->addServiceUUID(pScsService->getUUID());
  pAdvertising->setScanResponse(true);
  pAdvertising->start();

  Serial.println("Advertising Started");

  Serial.println("Starting Arduino BLE Client application...");
  BLEDevice::init("");

  BLEScan *pBLEScan = BLEDevice::getScan();
  pBLEScan->setAdvertisedDeviceCallbacks(new MyAdvertisedDeviceCallbacks());
  pBLEScan->setInterval(1349);
  pBLEScan->setWindow(449);
  pBLEScan->setActiveScan(true);
  pBLEScan->start(5, false);
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

  // --- Accumulate crank revolutions from cadence (Cycling Power) ---
  if (cadenceInstantaneous != 0 && (millis()) >= (lastRevolution + (60000 / cadenceInstantaneous)))
  {
    revolutions++;
    timestamp = (unsigned short)(((millis() * 1024) / 1000) % 65536);
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
      wheelTimestamp = (unsigned short)(((millis() * 1024) / 1000) % 65536);
      lastWheelRevolution = millis();
    }
  }

  // --- Re-broadcast every second while a client is connected ---
  if (millis() - lastNotify >= 1000)
  {
    if (pServer->getConnectedCount() > 0)
    {
      // Cycling Power Measurement (0x2A63)
      bleBuffer[0] = flags & 0xff;
      bleBuffer[1] = (flags >> 8) & 0xff;
      bleBuffer[2] = powerInstantaneous & 0xff;
      bleBuffer[3] = (powerInstantaneous >> 8) & 0xff;
      bleBuffer[4] = revolutions & 0xff;
      bleBuffer[5] = (revolutions >> 8) & 0xff;
      bleBuffer[6] = timestamp & 0xff;
      bleBuffer[7] = (timestamp >> 8) & 0xff;
      CyclingPowerMeasurement->setValue(bleBuffer, 8);
      CyclingPowerMeasurement->notify();

      // Speed/Cadence Measurement (0x2A5C)
      // flags: bit0 = wheel revolution present, bit1 = cadence present
      unsigned short scsFlags = 0;
      if (speedKmh > 0.0f)
        scsFlags |= 0x0001; // wheel revolution present
      if (cadenceInstantaneous > 0)
        scsFlags |= 0x0002; // cadence present

      scBuffer[0] = scsFlags & 0xff;
      scBuffer[1] = (scsFlags >> 8) & 0xff;
      scBuffer[2] = cadenceInstantaneous & 0xff;
      scBuffer[3] = (cadenceInstantaneous >> 8) & 0xff;
      scBuffer[4] = wheelRevolutions & 0xff;
      scBuffer[5] = (wheelRevolutions >> 8) & 0xff;
      scBuffer[6] = wheelTimestamp & 0xff;
      scBuffer[7] = (wheelTimestamp >> 8) & 0xff;
      SpeedCadenceMeasurement->setValue(scBuffer, 8);
      SpeedCadenceMeasurement->notify();

      lastNotify = millis();
    }
  }

  if (pServer->getConnectedCount() == 0)
  {
    powerInstantaneous = 0;
  }
}
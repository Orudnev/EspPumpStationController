/*
 * Blink
 * Turns on an LED on for one second,
 * then off for one second, repeatedly.
 */

#include <Arduino.h>
#include <WiFi.h>
#include <WebServer.h>
#include <Preferences.h>

// Set LED_BUILTIN if it is not defined by Arduino framework
#ifndef LED_BUILTIN
    #define LED_BUILTIN 2
#endif

//void setup()
// {
//   // initialize LED digital pin as an output.
//   pinMode(LED_BUILTIN, OUTPUT);
//   Serial.begin(115200);
// }

// void loop()
// {
//   // turn the LED on (HIGH is the voltage level)
//   digitalWrite(LED_BUILTIN, HIGH);
//   Serial.println("LED on");
//   // wait for a second
//   delay(3000);
//   // turn the LED off by making the voltage LOW
//   digitalWrite(LED_BUILTIN, LOW);
//   Serial.println("LED off");
//    // wait for a second
//   delay(1000);
// }


// ======================== ПИНЫ — проверьте под свою плату ========================
// Аналоговый вход давления (только входной пин!): GPIO34 = ADC1_CH6
static const int PIN_PRESSURE = 34;
// Цифровой вход датчика потока
static const int PIN_FLOW     = 27;  // при необходимости замените и/или включите подтяжку
// Управление SSR
static const int PIN_RELAY    = 26;

// Реле активно высоким уровнем? Если у вас инверсия — поставьте false
static const bool RELAY_ACTIVE_HIGH = true;

// ======================== Wi‑Fi ========================
// Попробовать подключиться к существующей сети; если не удалось — поднять точку доступа
static const char* STA_SSID = "YourWiFi";      // <- укажите SSID
static const char* STA_PASS = "YourPassword";  // <- укажите пароль
static const char* AP_SSID  = "PumpCtl-ESP32";
static const char* AP_PASS  = "12345678";      // минимум 8 символов

// ======================== Конфигурация ========================
struct Config {
  // Пороговые значения и тайминги (п.2):
  // Единицы измерения порогов должны соответствовать величине SP (см. расчёт ниже).
  float Pmin   = 0.6f;    // Минимальное давление — ниже считаем, что воды нет (3.1)
  float Pon    = 1.2f;    // Давление включения — нижняя граница гистерезиса (3.6)
  float Poff   = 2.0f;    // Давление отключения — верхняя граница (3.5)
  float Pdelta = 0.05f;   // Минимально допустимый прирост при старте (3.3)
  uint32_t Tf_ms   = 5000;   // Таймаут ожидания потока после старта (3.2), мс
  uint32_t Tbst_ms = 30000;  // Таймаут между включениями (3.2), мс

  // Калибровка давления из АЦП в единицы SP:
  // Вольтаж датчика: 0.5–4.5 В. На входе ESP32 максимум 3.3 В, поэтому предполагается делитель.
  // kDivider — во сколько раз понижается напряжение датчика на входе АЦП (пример: 4.5V -> 3.3V => kDivider ≈ 1.3636)
  float adcVref      = 3.3f;     // опорное напряжение АЦП ESP32
  float kDivider     = 1.3636f;  // коэффициент делителя (sensorV = adcV * kDivider)
  float sensorVmin   = 0.5f;     // напряжение датчика при 0 давлении
  float sensorVmax   = 4.5f;     // напряжение датчика при максимуме
  float pUnitsMax    = 10.0f;    // максимальное давление датчика в ваших единицах (напр., бар)
  bool  useEngineeringUnits = false; // false: SP в вольтах (sensorV); true: линейная конверсия в инженерные единицы
} cfg;

Preferences prefs;          // NVS для сохранения настроек
WebServer server(80);

// ======================== Состояние ========================
volatile bool relayOn = false;
uint32_t Tst = 0;              // время последнего включения насоса, мс (п.3)
float    SPlast = 0.0f;        // последний запомненный SP при старте (п.3.2)

// Состояние "ожидаем Tf после запуска" (п.3.2)
bool     inStartWait = false;
uint32_t startWaitUntil = 0;

// Опрос
static const uint32_t SAMPLE_PERIOD_MS = 200; // период опроса датчиков
uint32_t lastSample = 0;

// ======================== Утилиты времени (устойчивы к переполнению millis) ========================
static inline bool timePassed(uint32_t since, uint32_t interval) {
  return (uint32_t)(millis() - since) >= interval;
}

// ======================== АЦП и датчики ========================
float readPressureSP() {
  // Считываем ADC и пересчитываем в SP (вольты или инженерные единицы)
  uint16_t raw = analogRead(PIN_PRESSURE);
  float adcV = (raw / 4095.0f) * cfg.adcVref;
  float sensorV = adcV * cfg.kDivider; // напряжение на выходе датчика 0.5–4.5 В
  if (!cfg.useEngineeringUnits) {
    return sensorV; // SP в Вольтах (напряжение датчика)
  } else {
    // Линейная интерполяция в инженерные единицы (например, бар)
    float ratio = (sensorV - cfg.sensorVmin) / (cfg.sensorVmax - cfg.sensorVmin);
    ratio = constrain(ratio, 0.0f, 1.0f);
    return ratio * cfg.pUnitsMax; // SP в инженерных единицах
  }
}

int readFlowSF() {
  // 0 — потока нет, 1 — поток есть
  // При необходимости включите INPUT_PULLUP/INPUT_PULLDOWN и подберите инверсию
  return digitalRead(PIN_FLOW) ? 1 : 0;
}

void applyRelay(bool on) {
  relayOn = on;
  digitalWrite(PIN_RELAY, RELAY_ACTIVE_HIGH ? (on ? HIGH : LOW) : (on ? LOW : HIGH));
  if (on) Tst = millis();
}

// ======================== Веб-интерфейс ========================
String htmlIndex() {
    String s = R"rawliteral(
<!doctype html><html><head><meta charset='utf-8'><meta name='viewport' content='width=device-width, initial-scale=1'>
<title>ESP32 Pump Controller</title>
<style>
  body{font-family:system-ui;margin:20px}
  fieldset{margin:12px 0;padding:12px;border-radius:12px;border:1px solid #ccc}
  input,button{font-size:1rem;padding:6px 10px;margin:4px 0}
  label{display:block;margin-top:6px}
</style>
</head><body>
  <h1>Контроллер насосной станции</h1>
  <div id='status'>Нажмите «Обновить», чтобы получить данные</div>
  <button id="refreshBtn">Обновить</button>
  
  <fieldset><legend>Настройки</legend>
  <form id='cfgForm'>
    <label>Pmin <input name='Pmin' type='number' step='0.001'></label>
    <label>Pon <input name='Pon' type='number' step='0.001'></label>
    <label>Poff <input name='Poff' type='number' step='0.001'></label>
    <label>Pdelta <input name='Pdelta' type='number' step='0.001'></label>
    <label>Tf_ms <input name='Tf_ms' type='number' step='1'></label>
    <label>Tbst_ms <input name='Tbst_ms' type='number' step='1'></label>
    <label><input name='useEngineeringUnits' type='checkbox'> SP в инженерных единицах</label>
    <details><summary>Калибровка датчика</summary>
      <label>adcVref <input name='adcVref' type='number' step='0.01'></label>
      <label>kDivider <input name='kDivider' type='number' step='0.0001'></label>
      <label>sensorVmin <input name='sensorVmin' type='number' step='0.001'></label>
      <label>sensorVmax <input name='sensorVmax' type='number' step='0.001'></label>
      <label>pUnitsMax <input name='pUnitsMax' type='number' step='0.01'></label>
    </details>
    <button type='submit'>Сохранить</button>
  </form>
  </fieldset>

<script>
async function load(){
    const r = await fetch('/status');
    const j = await r.json();
    document.getElementById('status').innerHTML = `
      <fieldset><legend>Статус</legend>
      <div><b>SP</b>: ${j.SP.toFixed(3)} (${j.units})</div>
      <div><b>SF</b>: ${j.SF}</div>
      <div><b>Реле</b>: ${j.relayOn ? 'ON' : 'OFF'}</div>
      <div><b>Состояние</b>: ${j.stateNote}</div>
      <div><b>Tcurr</b>: ${j.Tcurr_ms} ms</div>
      <div><b>Tst</b>: ${j.Tst_ms} ms</div>
      </fieldset>`;
    const f = document.getElementById('cfgForm');
    for (const k of Object.keys(j.cfg)) {
      const el = f.querySelector(`[name=${k}]`);
      if (!el) continue;
      if (el.type === 'checkbox') el.checked = j.cfg[k];
      else el.value = j.cfg[k];
    }
}

document.getElementById('refreshBtn').addEventListener('click', load);

document.getElementById('cfgForm').addEventListener('submit', async (e)=>{
    e.preventDefault();
    const fd = new FormData(e.target);
    const obj = {};
    for (const [k,v] of fd.entries()) obj[k] = v;
    obj.useEngineeringUnits = !!e.target.useEngineeringUnits.checked;
    const r = await fetch('/save', {
      method:'POST',
      headers:{'Content-Type':'application/json'},
      body: JSON.stringify(obj)
    });
    alert(await r.text());
});
</script>
)rawliteral";

    return s;
}
 
void saveConfig() {
  prefs.begin("pumpctl", false);
  prefs.putFloat("Pmin", cfg.Pmin);
  prefs.putFloat("Pon", cfg.Pon);
  prefs.putFloat("Poff", cfg.Poff);
  prefs.putFloat("Pdelta", cfg.Pdelta);
  prefs.putUInt ("Tf_ms", cfg.Tf_ms);
  prefs.putUInt ("Tbst_ms", cfg.Tbst_ms);
  prefs.putFloat("adcVref", cfg.adcVref);
  prefs.putFloat("kDivider", cfg.kDivider);
  prefs.putFloat("sensorVmin", cfg.sensorVmin);
  prefs.putFloat("sensorVmax", cfg.sensorVmax);
  prefs.putFloat("pUnitsMax", cfg.pUnitsMax);
  prefs.putBool ("engUnits", cfg.useEngineeringUnits);
  prefs.end();
}

void loadConfig() {
  prefs.begin("pumpctl", true);
  cfg.Pmin   = prefs.getFloat("Pmin", cfg.Pmin);
  cfg.Pon    = prefs.getFloat("Pon", cfg.Pon);
  cfg.Poff   = prefs.getFloat("Poff", cfg.Poff);
  cfg.Pdelta = prefs.getFloat("Pdelta", cfg.Pdelta);
  cfg.Tf_ms   = prefs.getUInt ("Tf_ms", cfg.Tf_ms);
  cfg.Tbst_ms = prefs.getUInt ("Tbst_ms", cfg.Tbst_ms);
  cfg.adcVref    = prefs.getFloat("adcVref", cfg.adcVref);
  cfg.kDivider   = prefs.getFloat("kDivider", cfg.kDivider);
  cfg.sensorVmin = prefs.getFloat("sensorVmin", cfg.sensorVmin);
  cfg.sensorVmax = prefs.getFloat("sensorVmax", cfg.sensorVmax);
  cfg.pUnitsMax  = prefs.getFloat("pUnitsMax", cfg.pUnitsMax);
  cfg.useEngineeringUnits = prefs.getBool("engUnits", cfg.useEngineeringUnits);
  prefs.end();

  // Простейшая валидация/починка
  if (cfg.Pon >= cfg.Poff) cfg.Pon = cfg.Poff * 0.8f;
  if (cfg.Pmin >= cfg.Pon) cfg.Pmin = cfg.Pon * 0.8f;
}

void setupWeb() {
  server.on("/", HTTP_GET, [](){ server.send(200, "text/html", htmlIndex()); });

  server.on("/status", HTTP_GET, [](){
    float SP = readPressureSP();
    int   SF = readFlowSF();

    String json = "{";
    json += "\"SP\":" + String(SP, 5) + ",";
    json += "\"SF\":" + String(SF) + ",";
    json += "\"relayOn\":" + String(relayOn ? "true":"false") + ",";
    json += "\"Tcurr_ms\":" + String((unsigned long)millis()) + ",";
    json += "\"Tst_ms\":" + String((unsigned long)Tst) + ",";
    json += "\"units\":\"" + String(cfg.useEngineeringUnits?"eng":"V") + "\",";
    // Простая текстовая заметка о состоянии
    String note = inStartWait ? "ожидание Tf после старта [3.2]" : (relayOn ? "насос включен" : "насос выключен");
    json += "\"stateNote\":\"" + note + "\",";

    // Вернуть текущую конфигурацию для автозаполнения формы
    json += "\"cfg\":{";
    json += "\"Pmin\":" + String(cfg.Pmin,3) + ",";
    json += "\"Pon\":" + String(cfg.Pon,3) + ",";
    json += "\"Poff\":" + String(cfg.Poff,3) + ",";
    json += "\"Pdelta\":" + String(cfg.Pdelta,3) + ",";
    json += "\"Tf_ms\":" + String(cfg.Tf_ms) + ",";
    json += "\"Tbst_ms\":" + String(cfg.Tbst_ms) + ",";
    json += "\"adcVref\":" + String(cfg.adcVref,2) + ",";
    json += "\"kDivider\":" + String(cfg.kDivider,4) + ",";
    json += "\"sensorVmin\":" + String(cfg.sensorVmin,3) + ",";
    json += "\"sensorVmax\":" + String(cfg.sensorVmax,3) + ",";
    json += "\"pUnitsMax\":" + String(cfg.pUnitsMax,2) + ",";
    json += "\"useEngineeringUnits\":" + String(cfg.useEngineeringUnits?"true":"false");
    json += "}}";

    server.send(200, "application/json", json);
  });

  server.on("/save", HTTP_POST, [](){
    // Ожидаем JSON в теле запроса
    if (!server.hasArg("plain")) { server.send(400, "text/plain", "no body"); return; }
    String body = server.arg("plain");

    // Простейший парсер: ищем по ключевым словам (для компактности без внешних JSON-библиотек)
    auto getNum = [&](const char* key, float* outF, uint32_t* outU = nullptr){
      int i = body.indexOf(String("\"") + key + "\"");
      if (i < 0) return;
      i = body.indexOf(':', i);
      if (i < 0) return;
      int j = body.indexOf(',', i+1); if (j < 0) j = body.indexOf('}', i+1);
      if (j < 0) return;
      String v = body.substring(i+1, j);
      v.trim();
      if (outF) *outF = v.toFloat();
      if (outU) *outU = (uint32_t) v.toInt();
    };

    auto getBool = [&](const char* key, bool* outB){
      int i = body.indexOf(String("\"") + key + "\"");
      if (i < 0) return;
      i = body.indexOf(':', i);
      if (i < 0) return;
      int j = body.indexOf(',', i+1); if (j < 0) j = body.indexOf('}', i+1);
      if (j < 0) return;
      String v = body.substring(i+1, j);
      v.trim();
      *outB = v.indexOf('t') >= 0 || v == "1";
    };

    getNum("Pmin", &cfg.Pmin);
    getNum("Pon",  &cfg.Pon);
    getNum("Poff", &cfg.Poff);
    getNum("Pdelta", &cfg.Pdelta);
    getNum("Tf_ms", nullptr, &cfg.Tf_ms);
    getNum("Tbst_ms", nullptr, &cfg.Tbst_ms);
    getNum("adcVref", &cfg.adcVref);
    getNum("kDivider", &cfg.kDivider);
    getNum("sensorVmin", &cfg.sensorVmin);
    getNum("sensorVmax", &cfg.sensorVmax);
    getNum("pUnitsMax", &cfg.pUnitsMax);
    getBool("useEngineeringUnits", &cfg.useEngineeringUnits);

    // Валидация гистерезиса
    if (cfg.Pon >= cfg.Poff) cfg.Pon = cfg.Poff * 0.8f;
    if (cfg.Pmin >= cfg.Pon) cfg.Pmin = cfg.Pon * 0.8f;

    saveConfig();
    server.send(200, "text/plain", "saved");
  });

  server.begin();
}

// ======================== Управление по алгоритму (п.3) ========================
void controlLoopTick() {
  float SP = readPressureSP();
  int   SF = readFlowSF();
  uint32_t Tcurr = millis();

  // -------- [3.1] SP < Pmin: воды нет — отключить реле --------
  if (SP < cfg.Pmin) {
    applyRelay(false);
    inStartWait = false; // сбросить ожидание Tf
    return; // приоритетная аварийная ветка
  }

  // Если мы в фазе ожидания после старта (п.3.2) — просто ждем Tf, сохраняя реле включенным
  if (inStartWait) {
    if (!timePassed(Tst, cfg.Tf_ms)) {
      // продолжаем ожидание Tf (насос включен)
      return; // остальные пункты проверим после истечения Tf
    } else {
      // Tf истек — выходим из режима ожидания и продолжаем с 3.3+
      inStartWait = false;
    }
  }

  // -------- [3.2] (Pmin < SP < Poff) AND (SF == 0) AND (Tcurr - Tst > Tbst): запуск --------
  if (SP > cfg.Pmin && SP < cfg.Poff && SF == 0 && timePassed(Tst, cfg.Tbst_ms)) {
    SPlast = SP;                  // запомнить SPlast
    applyRelay(true);             // включить реле
    inStartWait = true;           // войти в фазу ожидания Tf
    // время последнего включения уже записано в applyRelay(true)
    return; // по п.3.2: "выждать таймаут Tf, затем перейти к нижеследующим пунктам"
  }

  // Начиная отсюда, либо мы не запускались, либо Tf уже истек и мы проверяем 3.3–3.6

  // -------- [3.3] (Pmin < SP < SPlast + Pdelta) AND (SF == 0): давление не растет — отключить --------
  if (SP > cfg.Pmin && SP < (SPlast + cfg.Pdelta) && SF == 0) {
    applyRelay(false);
    return;
  }

  // -------- [3.5] SP > Poff: давление набрано — отключить --------
  if (SP > cfg.Poff) {
    applyRelay(false);
    return;
  }

  // -------- [3.4] (Pmin < SP < Poff) AND (SF == 1): поток есть, давление набирается — держать ON --------
  if (SP > cfg.Pmin && SP < cfg.Poff && SF == 1) {
    applyRelay(true);
    return;
  }

  // -------- [3.6] (Pon < SP < Poff): давление приемлемое — ничего не делаем --------
  if (SP > cfg.Pon && SP < cfg.Poff) {
    // Ничего не делаем: оставляем текущее состояние реле без изменений
    return;
  }

  // В остальных случаях явного правила нет — ничего не меняем
}

void setup() {
  Serial.begin(115200);
  delay(200);

  // Настройка пинов
  pinMode(PIN_PRESSURE, INPUT);
  pinMode(PIN_FLOW, INPUT);
  pinMode(PIN_RELAY, OUTPUT);
  applyRelay(false);

  // Загрузка конфигурации
  loadConfig();

  // Wi‑Fi STA -> AP fallback
  WiFi.mode(WIFI_STA);
  WiFi.begin(STA_SSID, STA_PASS);
  Serial.print("Connecting WiFi");
  unsigned long t0 = millis();
  while (WiFi.status() != WL_CONNECTED && millis() - t0 < 10000) {
    delay(250); Serial.print('.');
  }
  if (WiFi.status() == WL_CONNECTED) {
    Serial.printf("\nWiFi STA: %s\nIP: %s\n", WiFi.SSID().c_str(), WiFi.localIP().toString().c_str());
  } else {
    Serial.println("\nSTA failed, starting AP...");
    WiFi.mode(WIFI_AP);
    WiFi.softAP(AP_SSID, AP_PASS);
    Serial.printf("AP SSID: %s, IP: %s\n", AP_SSID, WiFi.softAPIP().toString().c_str());
  }

  setupWeb();
}

void loop() {
  server.handleClient();
  if (timePassed(lastSample, SAMPLE_PERIOD_MS)) {
    lastSample = millis();
    controlLoopTick();
  }
}

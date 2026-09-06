#include <WiFi.h>
#include <WebServer.h>
#include <HardwareSerial.h>
#include <HTTPClient.h>

// ─── CONFIGURATION WiFi ──────────────────────────────────────
const char* WIFI_SSID     = "###";
const char* WIFI_PASSWORD = "###";

// ─── CONFIGURATION SUPABASE ──────────────────────────────────
const char* SUPABASE_URL    = "https://";
const char* SUPABASE_ANON   = "";
const char* DEVICE_SECRET   = "";
const unsigned long PUSH_INTERVAL_MS = 30000; // Envoi Supabase toutes les 30s

// ─── CONSTANTES MOTEUR Z14XEP ────────────────────────────────
// Utilisées pour les calculs de diagnostic et affichage
const int    Z14_RPM_IDLE      = 800;    // tr/min ralenti
const int    Z14_RPM_MAX       = 6200;   // tr/min max
const int    Z14_SPEED_MAX     = 178;    // km/h
const int    Z14_TEMP_NORMAL   = 90;     // °C température normale
const int    Z14_TEMP_HOT      = 105;    // °C seuil surchauffe
const float  Z14_BATT_MIN      = 11.5;  // V batterie minimale
const float  Z14_BATT_NORMAL   = 13.8;  // V batterie moteur tournant
const float  FUEL_TRIM_WARN    = 10.0;  // % seuil alerte STFT/LTFT
const float  O2_MIN            = 0.10;  // V sonde O2 min saine
const float  O2_MAX            = 0.90;  // V sonde O2 max saine

// ─── UART2 et WebServer ───────────────────────────────────────
HardwareSerial obdSerial(2);
WebServer server(80);

// ─── VARIABLES OBD EN TEMPS RÉEL ─────────────────────────────
int    rpm          = 0;
int    vehicleSpeed = 0;
int    coolantTemp  = 0;
float  battVoltage  = 0.0f;
int    engineLoad   = 0;
int    throttlePos  = 0;
int    map_kpa      = 0;
float  stft         = 0.0f;
float  ltft         = 0.0f;
float  o2voltage    = 0.0f;
int    ignTiming    = 0;

// Freeze Frame (données au moment du défaut)
int    ff_rpm       = 0;
int    ff_speed     = 0;
int    ff_temp      = 0;
int    ff_load      = 0;

// Statuts systèmes
bool   catalyst_ok  = true;
bool   o2_ok        = true;
bool   egr_ok       = true;
bool   misfire_ok   = true;

// Codes défauts
String dtcJson      = "[]";
String dtcRaw       = "";

// État OBD
bool   obdReady     = false;
int    obdBaud      = 0;
String obdProtocol  = "—";

// Flags NO DATA par PID (pour affichage dashboard)
bool nd_rpm  = false, nd_spd  = false, nd_tmp  = false;
bool nd_load = false, nd_tps  = false, nd_map  = false;
bool nd_ign  = false, nd_stft = false, nd_ltft = false;
bool nd_o2   = false, nd_bat  = false;

// Statistiques
int  totalReads = 0;
int  failReads  = 0;
unsigned long lastPush      = 0;
unsigned long lastLiveRead  = 0;
const unsigned long LIVE_READ_INTERVAL = 2500; // ms entre lectures OBD

// ─────────────────────────────────────────────────────────────
//  COMMUNICATION OBD
// ─────────────────────────────────────────────────────────────

// Envoyer une commande AT ou PID et attendre la réponse '>'
String sendOBD(const String& cmd, int timeout_ms = 2000) {
  // Vider le buffer
  while (obdSerial.available()) obdSerial.read();

  obdSerial.println(cmd);
  String resp = "";
  unsigned long t = millis();

  while (millis() - t < (unsigned long)timeout_ms) {
    while (obdSerial.available()) {
      char c = (char)obdSerial.read();
      if (c != '\r') resp += c;
    }
    if (resp.indexOf('>') >= 0) break;
    yield();
    delay(1);
  }

  resp.trim();

  // Supprimer l'écho de commande (ATE0 pas encore actif au premier appel)
  if (resp.startsWith(cmd)) {
    resp = resp.substring(cmd.length());
    resp.trim();
  }

  // Supprimer le prompt '>'
  int promptIdx = resp.lastIndexOf('>');
  if (promptIdx >= 0) {
    resp = resp.substring(0, promptIdx);
    resp.trim();
  }

  return resp;
}

// Parser un octet hexadécimal depuis la réponse OBD-II
// byteOffset = 0 → premier octet de données (après "41 PID")
int getObdByte(const String& raw, int byteOffset) {
  int idx = raw.indexOf("41");
  if (idx < 0) return -1;

  String s = raw.substring(idx);
  s.trim();

  // Tokeniser
  String tokens[20];
  int cnt = 0;
  String cur = "";

  for (int i = 0; i <= (int)s.length() && cnt < 20; i++) {
    char c = (i < (int)s.length()) ? s[i] : ' ';
    if (c == ' ' || c == '\n' || c == '\t' || c == '>') {
      if (cur.length() == 2) { // octet hex = exactement 2 chars
        tokens[cnt++] = cur;
        cur = "";
      } else if (cur.length() > 0) {
        cur = ""; // ignorer tokens malformés
      }
    } else if ((c >= '0' && c <= '9') || (c >= 'A' && c <= 'F') || (c >= 'a' && c <= 'f')) {
      cur += c;
      if (cur.length() > 2) cur = cur.substring(cur.length()-2); // garder 2 derniers
    }
  }

  // Structure : tokens[0]="41" tokens[1]=PID tokens[2+]=données
  int dataIdx = byteOffset + 2;
  if (dataIdx >= cnt) return -1;

  return (int)strtol(tokens[dataIdx].c_str(), NULL, 16);
}

// ─────────────────────────────────────────────────────────────
//  DÉTECTION BAUD RATE
// ─────────────────────────────────────────────────────────────

bool tryBaud(int baud) {
  Serial.println("\n[BAUD] ══ Test " + String(baud) + " bps ══");

  obdSerial.end();
  delay(200);
  obdSerial.begin(baud, SERIAL_8N1, 16, 17);
  delay(400);

  while (obdSerial.available()) obdSerial.read();

  // Envoyer ATZ (reset complet)
  obdSerial.println("ATZ");
  String r = "";
  unsigned long t = millis();
  while (millis() - t < 3500) {
    while (obdSerial.available()) r += (char)obdSerial.read();
    if (r.indexOf('>') >= 0) break;
    yield();
    delay(1);
  }
  r.trim();

  // Affichage diagnostic octet par octet
  Serial.print("[BAUD] Réponse (" + String(r.length()) + " car.): '");
  for (int i = 0; i < (int)r.length(); i++) {
    char c = r[i];
    if (c >= 32 && c < 127) Serial.print(c);
    else { Serial.print("\\x"); if((int)c<16) Serial.print("0"); Serial.print((int)c, HEX); }
  }
  Serial.println("'");

  if (r.length() > 0 && (
      r.indexOf("ELM") >= 0 ||
      r.indexOf(">")   >= 0 ||
      r.indexOf("OK")  >= 0 ||
      r.indexOf("ATZ") >= 0)) {
    Serial.println("[BAUD] ✓ Baud " + String(baud) + " VALIDÉ");
    return true;
  }
  Serial.println("[BAUD] ✗ Pas de réponse valide");
  return false;
}

// ─────────────────────────────────────────────────────────────
//  INITIALISATION ELM327
// ─────────────────────────────────────────────────────────────

void initELM() {
  Serial.println("\n[OBD] ════════════════════════════════════");
  Serial.println("[OBD]  INIT ELM327 — Opel Astra H Z14XEP");
  Serial.println("[OBD] ════════════════════════════════════");

  // Priorité 9600 (défaut SparkFun OBD-II UART)
  int bauds[] = { 9600, 38400, 57600, 115200 };
  bool baudOK = false;

  for (int i = 0; i < 4; i++) {
    if (tryBaud(bauds[i])) {
      obdBaud = bauds[i];
      baudOK  = true;
      break;
    }
    delay(600);
    yield();
  }

  if (!baudOK) {
    Serial.println("\n[OBD] ✗✗✗ AUCUN BAUD NE RÉPOND ✗✗✗");
    Serial.println("[OBD] CHECKLIST :");
    Serial.println("[OBD]   1. Clé contact → position ACC ou ON");
    Serial.println("[OBD]   2. Câble OBD-II bien branché");
    Serial.println("[OBD]   3. SparkFun TX→GPIO16 / RX→GPIO17");
    Serial.println("[OBD]   4. Essayer inverser TX/RX");
    Serial.println("[OBD]   5. VCC SparkFun = uniquement port OBD");
    return;
  }

  Serial.println("[OBD] ✓ Baud détecté : " + String(obdBaud) + " bps");
  delay(1000);

  // ── Configuration ELM327 ──────────────────────────────────
  Serial.println("\n[OBD] ── Configuration ELM327 ──");
  String r;

  r = sendOBD("ATZ", 3000);
  Serial.println("[OBD] ATZ (reset)    → '" + r + "'");
  delay(1500);

  r = sendOBD("ATE0", 1500);
  Serial.println("[OBD] ATE0 (écho OFF)→ '" + r + "' " + (r.indexOf("OK")>=0?"✓":"?"));

  r = sendOBD("ATL0", 1500);
  Serial.println("[OBD] ATL0 (LF OFF)  → '" + r + "' " + (r.indexOf("OK")>=0?"✓":"?"));

  r = sendOBD("ATS1", 1500);
  Serial.println("[OBD] ATS1 (espaces) → '" + r + "' " + (r.indexOf("OK")>=0?"✓":"?"));

  r = sendOBD("ATH0", 1500);
  Serial.println("[OBD] ATH0 (hdrs OFF)→ '" + r + "' " + (r.indexOf("OK")>=0?"✓":"?"));

  // Timeout ELM : 0x32 = 50 * 4ms = 200ms par message OBD
  // Pour ISO 9141-2 (lent) on utilise ATST50 = 320ms
  r = sendOBD("ATST50", 1500);
  Serial.println("[OBD] ATST50 (tmo)   → '" + r + "'");

  // Adaptive timing ON (ATAT1) — l'ELM adapte le délai automatiquement
  r = sendOBD("ATAT1", 1500);
  Serial.println("[OBD] ATAT1 (adapt.) → '" + r + "'");

  r = sendOBD("ATI", 1500);
  Serial.println("[OBD] ATI (version)  → '" + r + "'");

  // ── Détection protocole Z14XEP ───────────────────────────
  // ISO 9141-2 (ATSP3) = protocole standard Opel Astra H avant 2007
  // KWP2000 slow (ATSP4) = certaines variantes ou ECU mis à jour
  Serial.println("\n[OBD] ── Détection protocole ──");

  struct ProtoEntry { const char* cmd; const char* name; int timeout; };
  ProtoEntry protos[] = {
    { "ATSP3", "ISO 9141-2 (standard Z14XEP)", 6000 },
    { "ATSP4", "KWP2000 Slow Init",             5000 },
    { "ATSP5", "KWP2000 Fast Init",             4000 },
    { "ATSP0", "Auto-détection",                6000 }
  };

  for (int i = 0; i < 4; i++) {
    Serial.println("\n[OBD] → Essai : " + String(protos[i].name));
    r = sendOBD(protos[i].cmd, 1000);
    Serial.println("[OBD]   " + String(protos[i].cmd) + " → '" + r + "'");
    delay(500);

    // Tester la communication ECU avec PID 0100 (PIDs supportés)
    r = sendOBD("0100", protos[i].timeout);
    Serial.println("[OBD]   0100 → '" + r + "'");

    bool hasData = (r.indexOf("41") >= 0);
    bool hasError= (r.indexOf("NO DATA") >= 0 || r.indexOf("ERROR")  >= 0 ||
                    r.indexOf("UNABLE")  >= 0 || r.indexOf("BUS")    >= 0);

    if (hasData && !hasError) {
      obdReady    = true;
      obdProtocol = String(protos[i].name);
      Serial.println("[OBD] ✓✓✓ PROTOCOLE VALIDÉ : " + obdProtocol);

      // Afficher les PIDs supportés
      Serial.print("[OBD]   PIDs supportés (0100): ");
      // Décoder les 4 octets de bitmap
      for (int b = 0; b < 4; b++) {
        int bVal = getObdByte(r, b);
        if (bVal >= 0) {
          for (int bit = 7; bit >= 0; bit--) {
            int pidNum = b*8 + (7-bit) + 1;
            if (bVal & (1<<bit)) Serial.print("0x"+String(pidNum,HEX)+" ");
          }
        }
      }
      Serial.println();
      break;
    }

    if (hasError) Serial.println("[OBD]   → ERREUR / NO DATA");
    else          Serial.println("[OBD]   → Pas de réponse '41 00'");
    delay(800);
    yield();
  }

  if (!obdReady) {
    Serial.println("\n[OBD] ✗✗✗ AUCUN PROTOCOLE NE RÉPOND ✗✗✗");
    Serial.println("[OBD] Causes possibles :");
    Serial.println("[OBD]   1. Moteur éteint → démarrer le moteur");
    Serial.println("[OBD]   2. Clé en ACC seulement → mettre sur ON");
    Serial.println("[OBD]   3. TX/RX inversés → échanger GPIO16/17");
    Serial.println("[OBD]   4. Clone ELM327 défectueux");
    Serial.println("[OBD]   5. Fusible OBD-II véhicule grillé");
  }
}

// ─────────────────────────────────────────────────────────────
//  LECTURE D'UN PID AVEC DIAGNOSTIC
// ─────────────────────────────────────────────────────────────

String readPID(const String& pid, const String& label, bool* ndFlag = nullptr) {
  totalReads++;
  String r = sendOBD(pid, 2200);

  bool isNoData = (r.length() == 0          ||
                   r.indexOf("41")  < 0     ||
                   r.indexOf("NO DATA") >= 0||
                   r.indexOf("ERROR")   >= 0||
                   r.indexOf("UNABLE")  >= 0);

  if (ndFlag) *ndFlag = isNoData;

  if (isNoData) {
    failReads++;
    Serial.println("[PID] ✗ " + label + " (" + pid + ") → NO DATA  ["
                   + String(failReads) + "/" + String(totalReads) + " échecs]");
  } else {
    Serial.println("[PID] ✓ " + label + " (" + pid + ") → '" + r + "'");
  }
  return r;
}

// ─────────────────────────────────────────────────────────────
//  LECTURE DONNÉES TEMPS RÉEL — Z14XEP PIDs
// ─────────────────────────────────────────────────────────────

void readLiveData() {
  if (!obdReady) {
    Serial.println("[OBD] ✗ OBD non prêt — lecture ignorée");
    return;
  }

  Serial.println("\n[OBD] ───────── Lecture live Z14XEP ─────────");
  String r;
  int A, B;

  // ── RPM — PID 010C ── Formule: ((A×256)+B)/4
  r = readPID("010C", "RPM", &nd_rpm);
  if (!nd_rpm) {
    A = getObdByte(r, 0); B = getObdByte(r, 1);
    if (A >= 0 && B >= 0) {
      rpm = ((A * 256) + B) / 4;
      // Sanity check Z14XEP (0–6200 tr/min)
      if (rpm < 0 || rpm > 7000) rpm = 0;
      Serial.println("[OBD]   RPM = " + String(rpm) + " tr/min"
                     + (rpm < Z14_RPM_IDLE ? " ⚠ sous ralenti" : ""));
    }
  }

  // ── Vitesse — PID 010D ── Formule: A km/h
  r = readPID("010D", "Vitesse", &nd_spd);
  if (!nd_spd) {
    A = getObdByte(r, 0);
    if (A >= 0) {
      vehicleSpeed = A;
      if (vehicleSpeed > 200) vehicleSpeed = 0; // sanity
      Serial.println("[OBD]   Vitesse = " + String(vehicleSpeed) + " km/h");
    }
  }

  // ── Température liquide — PID 0105 ── Formule: A-40 °C
  r = readPID("0105", "Temp. moteur", &nd_tmp);
  if (!nd_tmp) {
    A = getObdByte(r, 0);
    if (A >= 0) {
      coolantTemp = A - 40;
      String warn = "";
      if (coolantTemp < 60) warn = " (chauffe en cours)";
      else if (coolantTemp >= Z14_TEMP_HOT) warn = " ⚠ SURCHAUFFE !";
      Serial.println("[OBD]   Temp = " + String(coolantTemp) + " °C" + warn);
    }
  }

  // ── Charge moteur — PID 0104 ── Formule: (A×100)/255 %
  r = readPID("0104", "Charge moteur", &nd_load);
  if (!nd_load) {
    A = getObdByte(r, 0);
    if (A >= 0) {
      engineLoad = (A * 100) / 255;
      Serial.println("[OBD]   Charge = " + String(engineLoad) + " %");
    }
  }

  // ── Papillon TPS — PID 0111 ── Formule: (A×100)/255 %
  r = readPID("0111", "Papillon (TPS)", &nd_tps);
  if (!nd_tps) {
    A = getObdByte(r, 0);
    if (A >= 0) {
      throttlePos = (A * 100) / 255;
      Serial.println("[OBD]   Papillon = " + String(throttlePos) + " %");
    }
  }

  // ── Pression admission MAP — PID 010B ── Formule: A kPa
  // Z14XEP MPI atmosphérique : ~30kPa ralenti, ~100kPa pleine charge
  r = readPID("010B", "MAP (admission)", &nd_map);
  if (!nd_map) {
    A = getObdByte(r, 0);
    if (A >= 0) {
      map_kpa = A;
      Serial.println("[OBD]   MAP = " + String(map_kpa) + " kPa"
                     + (map_kpa > 100 ? " (pression atmosphérique)" : ""));
    }
  }

  // ── Avance allumage — PID 010E ── Formule: (A/2)-64 °
  // Z14XEP : ~10° ralenti, avance réduite si cliquetis
  r = readPID("010E", "Avance allumage", &nd_ign);
  if (!nd_ign) {
    A = getObdByte(r, 0);
    if (A >= 0) {
      ignTiming = (A / 2) - 64;
      Serial.println("[OBD]   Avance = " + String(ignTiming) + " °"
                     + (ignTiming < 0 ? " ⚠ retard allumage !" : ""));
    }
  }

  // ── STFT — PID 0106 ── Formule: ((A-128)×100)/128 %
  // Normal : ±5% | Alerte : >±10% = problème injection/sonde O2
  r = readPID("0106", "STFT (correction CT)", &nd_stft);
  if (!nd_stft) {
    A = getObdByte(r, 0);
    if (A >= 0) {
      stft = ((A - 128) * 100.0f) / 128.0f;
      String warn = "";
      if (stft >  FUEL_TRIM_WARN) warn = " ⚠ mélange pauvre !";
      if (stft < -FUEL_TRIM_WARN) warn = " ⚠ mélange riche !";
      Serial.println("[OBD]   STFT = " + String(stft, 1) + " %" + warn);
    }
  }

  // ── LTFT — PID 0107 ── Formule: ((A-128)×100)/128 %
  r = readPID("0107", "LTFT (correction LT)", &nd_ltft);
  if (!nd_ltft) {
    A = getObdByte(r, 0);
    if (A >= 0) {
      ltft = ((A - 128) * 100.0f) / 128.0f;
      String warn = "";
      if (ltft >  FUEL_TRIM_WARN) warn = " ⚠ fuite admission/injec. !";
      if (ltft < -FUEL_TRIM_WARN) warn = " ⚠ richesse excessive !";
      Serial.println("[OBD]   LTFT = " + String(ltft, 1) + " %" + warn);
    }
  }

  // ── Sonde O2 Banque 1 Capteur 1 — PID 0114 ── Formule: A×0.005 V
  // Z14XEP : sonde en amont catalyseur — doit osciller 0.1V–0.9V
  r = readPID("0114", "O2 amont (B1S1)", &nd_o2);
  if (!nd_o2) {
    A = getObdByte(r, 0);
    if (A >= 0) {
      o2voltage = A * 0.005f;
      bool o2sane = (o2voltage >= O2_MIN && o2voltage <= O2_MAX);
      o2_ok = o2sane;
      Serial.println("[OBD]   O2 = " + String(o2voltage, 3) + " V"
                     + (!o2sane ? " ⚠ hors plage !" : ""));
    }
  }

  // ── Tension batterie — PID 0142 (ou 0146) ─────────────────
  // REMARQUE : PID 0142 rarement supporté sur Z14XEP
  // Alternative : ATRV lit la tension directement depuis l'ELM327
  r = readPID("0142", "Batterie (module ECU)", &nd_bat);
  if (!nd_bat) {
    A = getObdByte(r, 0); B = getObdByte(r, 1);
    if (A >= 0 && B >= 0) {
      battVoltage = ((A * 256) + B) / 1000.0f;
      Serial.println("[OBD]   Batterie = " + String(battVoltage, 2) + " V (PID 0142)");
    }
  } else {
    // Fallback 1 : PID 0146
    Serial.println("[OBD]   0142 NO DATA → essai 0146...");
    r = sendOBD("0146", 2000);
    A = getObdByte(r, 0); B = getObdByte(r, 1);
    if (A >= 0 && B >= 0) {
      battVoltage = ((A * 256) + B) / 1000.0f;
      nd_bat = false;
      Serial.println("[OBD]   Batterie = " + String(battVoltage, 2) + " V (PID 0146)");
    } else {
      // Fallback 2 : ATRV (lecture directe ELM327 — toujours disponible)
      Serial.println("[OBD]   0146 NO DATA → essai ATRV (ELM direct)...");
      String rv = sendOBD("ATRV", 1500);
      Serial.println("[OBD]   ATRV → '" + rv + "'");
      // Format réponse : "12.6V"
      rv.replace("V", "");
      rv.trim();
      float v = rv.toFloat();
      if (v > 6.0f && v < 18.0f) {
        battVoltage = v;
        nd_bat = false;
        Serial.println("[OBD]   Batterie = " + String(battVoltage, 2) + " V (ATRV)");
      } else {
        Serial.println("[OBD]   Batterie : aucune source disponible");
      }
    }
  }

  // ── Statut allumage / ratés — PID 0101 bit 28 ────────────
  // Bit 28 du PID 0101 = misfire detected
  r = sendOBD("0101", 2000);
  if (r.indexOf("41") >= 0) {
    int byteB = getObdByte(r, 1);
    if (byteB >= 0) {
      misfire_ok = !((byteB >> 4) & 0x01); // bit 4 d'octet B
    }
  }

  // ── Résumé lecture ────────────────────────────────────────
  int ok = totalReads - failReads;
  Serial.println("[OBD] ─── Fin lecture │ ✓ " + String(ok) + "/" + String(totalReads) + " ───");

  if (totalReads > 10) {
    int pct = (failReads * 100) / totalReads;
    if (pct > 40) {
      Serial.println("[OBD] ⚠ " + String(pct) + "% d'échecs → ECU lent ou protocole incorrect");
      Serial.println("[OBD]   → Essayer ATST64 (timeout plus long)");
    }
  }
}

// ─────────────────────────────────────────────────────────────
//  LECTURE CODES DÉFAUTS (DTC)
// ─────────────────────────────────────────────────────────────

void readDTC() {
  if (!obdReady) return;
  Serial.println("\n[DTC] ── Lecture DTC (mode 03) ──");

  dtcRaw  = sendOBD("03", 5000);
  dtcJson = "[";
  bool first = true;

  Serial.println("[DTC] Réponse brute: '" + dtcRaw + "'");

  if (dtcRaw.length() == 0 || dtcRaw.indexOf("43") < 0 ||
      dtcRaw.indexOf("NO DATA") >= 0) {
    dtcJson += "]";
    Serial.println("[DTC] ✓ Aucun code défaut");
    return;
  }

  // Parser la réponse mode 43
  int idx = dtcRaw.indexOf("43");
  if (idx < 0) { dtcJson += "]"; return; }

  String data = dtcRaw.substring(idx + 3);
  data.trim();

  Serial.println("[DTC] Données DTC : '" + data + "'");

  while (data.length() >= 5) {
    String bA_str = data.substring(0, 2);
    String bB_str = data.substring(3, 5);
    data = (data.length() > 6) ? data.substring(6) : "";
    data.trim();

    int byteA = (int)strtol(bA_str.c_str(), NULL, 16);
    int byteB = (int)strtol(bB_str.c_str(), NULL, 16);
    if (byteA == 0 && byteB == 0) continue;

    char typeChar;
    switch ((byteA >> 6) & 0x03) { // bits 7-6 du premier octet
      case 0:  typeChar = 'P'; break;
      case 1:  typeChar = 'C'; break;
      case 2:  typeChar = 'B'; break;
      default: typeChar = 'U'; break;
    }
    int digits = ((byteA & 0x3F) << 8) | byteB;
    char dtcStr[8];
    snprintf(dtcStr, sizeof(dtcStr), "%c%04X", typeChar, digits);

    if (!first) dtcJson += ",";
    dtcJson += "\"" + String(dtcStr) + "\"";
    first = false;
    Serial.println("[DTC] ⚠ DTC trouvé : " + String(dtcStr)
                   + " → " + dtcDescription(String(dtcStr)));
  }

  dtcJson += "]";
  Serial.println("[DTC] JSON final : " + dtcJson);
}

// ─────────────────────────────────────────────────────────────
//  LECTURE FREEZE FRAME
// ─────────────────────────────────────────────────────────────

void readFreezeFrame() {
  if (!obdReady) return;
  Serial.println("\n[FF] ── Lecture Freeze Frame (mode 02) ──");
  String r;
  int A, B;

  // NOTE : commande mode 02 sans espace entre PID et frame#
  r = sendOBD("020C00", 3000);
  Serial.println("[FF] 020C00 (RPM)   → '" + r + "'");
  A = getObdByte(r, 0); B = getObdByte(r, 1);
  if (A >= 0 && B >= 0) ff_rpm = ((A*256)+B)/4;

  r = sendOBD("020D00", 3000);
  Serial.println("[FF] 020D00 (SPD)   → '" + r + "'");
  A = getObdByte(r, 0);
  if (A >= 0) ff_speed = A;

  r = sendOBD("020500", 3000);
  Serial.println("[FF] 020500 (TEMP)  → '" + r + "'");
  A = getObdByte(r, 0);
  if (A >= 0) ff_temp = A - 40;

  r = sendOBD("020400", 3000);
  Serial.println("[FF] 020400 (LOAD)  → '" + r + "'");
  A = getObdByte(r, 0);
  if (A >= 0) ff_load = (A * 100) / 255;

  Serial.println("[FF] Résultat → RPM=" + String(ff_rpm)
                 + " SPD="  + String(ff_speed)
                 + " TMP="  + String(ff_temp)
                 + " LOAD=" + String(ff_load));
}

// ─────────────────────────────────────────────────────────────
//  DESCRIPTIONS DTC — Z14XEP & OBD-II standard
// ─────────────────────────────────────────────────────────────

String dtcDescription(const String& code) {
  // Carburant / injection
  if (code=="P0171") return "Mélange pauvre — Banque 1 (fuite admission ?)";
  if (code=="P0172") return "Mélange riche — Banque 1 (injecteur fuite ?)";
  if (code=="P0174") return "Mélange pauvre — Banque 2";
  if (code=="P0175") return "Mélange riche — Banque 2";
  // Injecteurs Z14XEP (4 cylindres)
  if (code=="P0201") return "Circuit injecteur cylindre 1";
  if (code=="P0202") return "Circuit injecteur cylindre 2";
  if (code=="P0203") return "Circuit injecteur cylindre 3";
  if (code=="P0204") return "Circuit injecteur cylindre 4";
  if (code=="P0261") return "Injecteur 1 — court-circuit masse";
  if (code=="P0262") return "Injecteur 1 — court-circuit +12V";
  if (code=="P0264") return "Injecteur 2 — court-circuit masse";
  if (code=="P0267") return "Injecteur 3 — court-circuit masse";
  if (code=="P0270") return "Injecteur 4 — court-circuit masse";
  // Ratés allumage
  if (code=="P0300") return "Ratés allumage multiples (bougies/bobine ?)";
  if (code=="P0301") return "Raté allumage cylindre 1";
  if (code=="P0302") return "Raté allumage cylindre 2";
  if (code=="P0303") return "Raté allumage cylindre 3";
  if (code=="P0304") return "Raté allumage cylindre 4";
  // Sonde O2
  if (code=="P0130") return "O2 Banque 1 S1 — hors plage";
  if (code=="P0131") return "O2 Banque 1 S1 — tension basse";
  if (code=="P0132") return "O2 Banque 1 S1 — tension haute";
  if (code=="P0133") return "O2 Banque 1 S1 — réponse lente (vieille sonde ?)";
  if (code=="P0134") return "O2 Banque 1 S1 — aucune activité";
  if (code=="P0136") return "O2 Banque 1 S2 — circuit ouvert";
  if (code=="P0141") return "O2 Banque 1 S2 — chauffage défectueux";
  // Catalyseur
  if (code=="P0420") return "Efficacité catalyseur faible Banque 1";
  // Capteurs admission
  if (code=="P0100") return "Débitm. MAF — circuit";
  if (code=="P0101") return "Débitm. MAF — hors plage";
  if (code=="P0102") return "Débitm. MAF — signal bas";
  if (code=="P0103") return "Débitm. MAF — signal haut";
  if (code=="P0106") return "MAP — hors plage";
  if (code=="P0107") return "MAP — signal bas";
  if (code=="P0108") return "MAP — signal haut";
  if (code=="P0112") return "Capteur IAT — signal bas";
  if (code=="P0113") return "Capteur IAT — signal haut";
  if (code=="P0115") return "Capteur température — circuit";
  if (code=="P0116") return "Capteur température — hors plage";
  if (code=="P0117") return "Capteur température — signal bas";
  if (code=="P0118") return "Capteur température — signal haut";
  // Thermostat
  if (code=="P0128") return "Thermostat — temp. sous la plage (thermostat HS ?)";
  // Papillon
  if (code=="P0120") return "Capteur TPS — circuit";
  if (code=="P0121") return "Capteur TPS — hors plage";
  if (code=="P0122") return "Capteur TPS — signal bas";
  if (code=="P0123") return "Capteur TPS — signal haut";
  // Capteur arbre à cames / vilebrequin
  if (code=="P0335") return "Capteur vilebrequin (CKP) — circuit";
  if (code=="P0336") return "Capteur vilebrequin (CKP) — plage/performance";
  if (code=="P0340") return "Capteur arbre à cames (CMP) — circuit";
  if (code=="P0341") return "Capteur arbre à cames (CMP) — plage/performance";
  // Ralenti
  if (code=="P0505") return "Régulation ralenti — panne générale";
  if (code=="P0506") return "Régulation ralenti — régime bas";
  if (code=="P0507") return "Régulation ralenti — régime haut";
  // EGR
  if (code=="P0400") return "Système EGR — débit insuffisant";
  if (code=="P0401") return "EGR — débit faible détecté";
  if (code=="P0402") return "EGR — débit excessif";
  if (code=="P0403") return "Circuit commande EGR";
  // Tension
  if (code=="P0560") return "Tension système — circuit";
  if (code=="P0562") return "Tension système — basse (<10V ?)";
  if (code=="P0563") return "Tension système — haute (>16V ?)";
  // ECM
  if (code=="P0600") return "Lien de communication ECM";
  if (code=="P0601") return "Mémoire ECM — erreur lecture";
  if (code=="P0606") return "Processeur interne ECM — défaut";
  // Allumage
  if (code=="P0350") return "Circuit bobine d'allumage";
  if (code=="P0351") return "Bobine cyl. 1 — circuit primaire";
  if (code=="P0352") return "Bobine cyl. 2 — circuit primaire";
  if (code=="P0353") return "Bobine cyl. 3 — circuit primaire";
  if (code=="P0354") return "Bobine cyl. 4 — circuit primaire";

  return "Code inconnu — consulter ELM327 wiki ou AllDatDIY";
}

// ─────────────────────────────────────────────────────────────
//  DASHBOARD HTML — Interface Web
// ─────────────────────────────────────────────────────────────

const char DASHBOARD_HTML[] PROGMEM = R"HTMLEOF(
<!DOCTYPE html>
<html lang="fr">
<head>
<meta charset="UTF-8">
<meta name="viewport" content="width=device-width,initial-scale=1">
<title>Opel Astra H 1.4 — OBD-II Live</title>
<style>
:root {
  --bg: #0f1117;
  --surface: #1a1d27;
  --border: #2a2d3a;
  --accent: #f97316;
  --green: #22c55e;
  --red: #ef4444;
  --yellow: #eab308;
  --blue: #3b82f6;
  --text: #e2e8f0;
  --muted: #64748b;
  --card-r: 12px;
}
* { box-sizing:border-box; margin:0; padding:0; }
body { font-family: 'Segoe UI', system-ui, sans-serif; background:var(--bg); color:var(--text); padding:16px; min-height:100vh; }

/* Header */
.header { display:flex; align-items:flex-start; justify-content:space-between; margin-bottom:18px; }
.header-left h1 { font-size:17px; font-weight:700; letter-spacing:-.02em; color:var(--text); }
.header-left .sub { font-size:11px; color:var(--muted); margin-top:3px; display:flex; align-items:center; gap:6px; }
.live-dot { width:7px; height:7px; border-radius:50%; background:var(--green); animation:pulse 1.6s ease-in-out infinite; display:inline-block; }
@keyframes pulse { 0%,100%{opacity:1} 50%{opacity:.25} }
.obd-info { font-size:10px; color:var(--muted); margin-top:5px; font-family:monospace; }
.badge { font-size:11px; padding:4px 12px; border-radius:20px; font-weight:600; border:1px solid; }
.badge.ok  { border-color:var(--green); color:var(--green); background:rgba(34,197,94,.08); }
.badge.err { border-color:var(--red);   color:var(--red);   background:rgba(239,68,68,.08); }

/* Layout */
.grid3 { display:grid; grid-template-columns:repeat(3,1fr); gap:12px; margin-bottom:12px; }
.grid3b { display:grid; grid-template-columns:repeat(3,1fr); gap:12px; margin-bottom:12px; }
.grid2  { display:grid; grid-template-columns:2fr 1fr; gap:12px; margin-bottom:12px; }

/* Cards */
.card { background:var(--surface); border:1px solid var(--border); border-radius:var(--card-r); padding:14px; }
.card-title { font-size:9px; font-weight:700; letter-spacing:.1em; text-transform:uppercase; color:var(--muted); margin-bottom:14px; }

/* Gauge */
.gauge-wrap { display:flex; flex-direction:column; align-items:center; }
.gauge-val  { font-size:30px; font-weight:700; line-height:1; color:var(--text); margin-top:-10px; }
.gauge-unit { font-size:10px; color:var(--muted); margin-top:6px; }

/* Rows */
.row  { display:flex; align-items:center; gap:8px; padding:7px 0; border-bottom:1px solid var(--border); font-size:12px; }
.row:last-child { border-bottom:none; }
.dot  { width:8px; height:8px; border-radius:50%; flex-shrink:0; }
.dot.ok { background:var(--green); }
.dot.w  { background:var(--yellow); }
.dot.e  { background:var(--red); }
.dot.nd { background:var(--border); }
.row-name { flex:1; color:var(--text); }
.row-val  { color:var(--muted); font-family:monospace; font-size:11px; }
.row-val.nd { color:var(--border); font-style:italic; }
.row-val.warn { color:var(--yellow); }
.row-val.alert { color:var(--red); font-weight:700; }

/* DTC */
.dtc-item { background:rgba(239,68,68,.07); border:1px solid rgba(239,68,68,.2); border-radius:8px; padding:9px 11px; margin-bottom:8px; }
.dtc-code { font-family:monospace; color:var(--red); font-weight:700; font-size:13px; }
.dtc-desc { font-size:10px; color:var(--muted); margin-top:2px; }
.dtc-ok   { font-size:12px; color:var(--green); }

/* Freeze Frame */
.ff-grid { display:grid; grid-template-columns:1fr 1fr; gap:10px; margin-top:10px; }
.ff-lbl  { font-size:9px; color:var(--muted); margin-bottom:2px; text-transform:uppercase; letter-spacing:.06em; }
.ff-val  { font-size:22px; font-weight:600; color:var(--text); }

/* System Status */
.sys-row  { display:flex; align-items:center; gap:8px; padding:8px 0; border-bottom:1px solid var(--border); }
.sys-row:last-child { border-bottom:none; }
.sys-name { flex:1; font-size:12px; }
.sys-st   { font-size:11px; }

/* Alerts */
.alert-box { background:rgba(234,179,8,.06); border:1px solid rgba(234,179,8,.3); border-radius:8px; padding:9px 12px; margin-bottom:10px; font-size:11px; color:var(--yellow); }

/* Button */
.btn { width:100%; margin-top:10px; padding:9px; background:transparent; border:1px solid var(--border); border-radius:8px; font-size:12px; color:var(--text); cursor:pointer; transition:.15s; }
.btn:hover { border-color:var(--accent); color:var(--accent); }

/* Footer */
#footer { font-size:10px; color:var(--muted); text-align:right; margin-top:10px; font-family:monospace; }

/* Responsive */
@media(max-width:640px) {
  .grid3,.grid3b { grid-template-columns:1fr; }
  .grid2 { grid-template-columns:1fr; }
}

/* Accent bar */
.accent-bar { height:3px; background:linear-gradient(90deg,var(--accent),#fbbf24); border-radius:2px; margin-bottom:18px; }
</style>
</head>
<body>

<div class="accent-bar"></div>

<div class="header">
  <div class="header-left">
    <h1>🔧 Opel Astra H 1.4 MPI (Z14XEP) — OBD-II</h1>
    <div class="sub"><span class="live-dot"></span>Données en direct · ESP32 WiFi</div>
    <div class="obd-info" id="obd-info">Initialisation...</div>
  </div>
  <span class="badge ok" id="badge">●&nbsp;Connecté</span>
</div>

<!-- Jauges principales -->
<div class="grid3">
  <div class="card gauge-wrap">
    <div class="card-title">Régime moteur</div>
    <svg width="150" height="82" viewBox="0 0 150 82" overflow="visible">
      <path d="M15 74 A60 60 0 0 1 135 74" fill="none" stroke="#2a2d3a" stroke-width="9" stroke-linecap="round"/>
      <path id="g-rpm" d="M15 74 A60 60 0 0 1 135 74" fill="none" stroke="#f97316" stroke-width="9" stroke-linecap="round" stroke-dasharray="188" stroke-dashoffset="188" style="transition:stroke-dashoffset .5s ease"/>
    </svg>
    <div class="gauge-val" id="v-rpm">—</div>
    <div class="gauge-unit">tr/min · max 6200</div>
  </div>
  <div class="card gauge-wrap">
    <div class="card-title">Vitesse</div>
    <svg width="150" height="82" viewBox="0 0 150 82" overflow="visible">
      <path d="M15 74 A60 60 0 0 1 135 74" fill="none" stroke="#2a2d3a" stroke-width="9" stroke-linecap="round"/>
      <path id="g-spd" d="M15 74 A60 60 0 0 1 135 74" fill="none" stroke="#22c55e" stroke-width="9" stroke-linecap="round" stroke-dasharray="188" stroke-dashoffset="188" style="transition:stroke-dashoffset .5s ease"/>
    </svg>
    <div class="gauge-val" id="v-spd">—</div>
    <div class="gauge-unit">km/h · max 178</div>
  </div>
  <div class="card gauge-wrap">
    <div class="card-title">Température moteur</div>
    <svg width="150" height="82" viewBox="0 0 150 82" overflow="visible">
      <path d="M15 74 A60 60 0 0 1 135 74" fill="none" stroke="#2a2d3a" stroke-width="9" stroke-linecap="round"/>
      <path id="g-tmp" d="M15 74 A60 60 0 0 1 135 74" fill="none" stroke="#3b82f6" stroke-width="9" stroke-linecap="round" stroke-dasharray="188" stroke-dashoffset="188" style="transition:stroke-dashoffset .5s ease,stroke .5s ease"/>
    </svg>
    <div class="gauge-val" id="v-tmp">—</div>
    <div class="gauge-unit">°C · normale 90°</div>
  </div>
</div>

<!-- Paramètres moteur & Carburant & DTC -->
<div class="grid3b">
  <div class="card">
    <div class="card-title">Paramètres moteur</div>
    <div class="row"><span class="dot ok" id="d-load"></span><span class="row-name">Charge moteur</span><span class="row-val" id="v-load">—</span></div>
    <div class="row"><span class="dot ok" id="d-map"></span><span class="row-name">Pression admission</span><span class="row-val" id="v-map">—</span></div>
    <div class="row"><span class="dot ok" id="d-ign"></span><span class="row-name">Avance allumage</span><span class="row-val" id="v-ign">—</span></div>
    <div class="row"><span class="dot ok" id="d-o2"></span><span class="row-name">Sonde O₂ amont</span><span class="row-val" id="v-o2">—</span></div>
    <div class="row"><span class="dot ok" id="d-tps"></span><span class="row-name">Papillon (TPS)</span><span class="row-val" id="v-tps">—</span></div>
  </div>
  <div class="card">
    <div class="card-title">Alimentation &amp; Carburant</div>
    <div class="row"><span class="dot ok" id="d-bat"></span><span class="row-name">Tension batterie</span><span class="row-val" id="v-bat">—</span></div>
    <div class="row"><span class="dot ok" id="d-stft"></span><span class="row-name">Correction CT (STFT)</span><span class="row-val" id="v-stft">—</span></div>
    <div class="row"><span class="dot ok" id="d-ltft"></span><span class="row-name">Correction LT (LTFT)</span><span class="row-val" id="v-ltft">—</span></div>
    <div class="row"><span class="dot ok"></span><span class="row-name">Pression carburant</span><span class="row-val">3.0 bar</span></div>
    <div class="row"><span class="dot ok"></span><span class="row-name">Ralenti nominal</span><span class="row-val">800 tr/min</span></div>
  </div>
  <div class="card">
    <div class="card-title">Codes défauts (DTC)</div>
    <div id="dtc-box"><div style="font-size:11px;color:var(--muted)">Lecture en cours...</div></div>
    <button class="btn" onclick="clearDTC()">🗑 Effacer les codes DTC</button>
  </div>
</div>

<!-- Freeze Frame & Statut systèmes -->
<div class="grid2">
  <div class="card">
    <div class="card-title">Données gelées au défaut (Freeze Frame)</div>
    <div class="ff-grid">
      <div><div class="ff-lbl">RPM au défaut</div><div class="ff-val" id="ff-r">—</div></div>
      <div><div class="ff-lbl">Vitesse au défaut</div><div class="ff-val" id="ff-s">—</div></div>
      <div><div class="ff-lbl">Temp. au défaut</div><div class="ff-val" id="ff-t">—</div></div>
      <div><div class="ff-lbl">Charge au défaut</div><div class="ff-val" id="ff-l">—</div></div>
    </div>
  </div>
  <div class="card">
    <div class="card-title">Statut systèmes OBD</div>
    <div class="sys-row"><span class="dot ok" id="s-mis"></span><span class="sys-name">Ratés allumage</span><span class="sys-st" id="t-mis">OK</span></div>
    <div class="sys-row"><span class="dot ok" id="s-cat"></span><span class="sys-name">Catalyseur</span><span class="sys-st" id="t-cat">OK</span></div>
    <div class="sys-row"><span class="dot ok" id="s-o2"></span><span class="sys-name">Sonde O₂</span><span class="sys-st" id="t-o2">OK</span></div>
    <div class="sys-row"><span class="dot ok" id="s-egr"></span><span class="sys-name">Système EGR</span><span class="sys-st" id="t-egr">OK</span></div>
  </div>
</div>

<div id="alert-box" style="display:none" class="alert-box"></div>
<div id="footer">Connexion en cours...</div>

<script>
const HOST = window.location.hostname;

function arc(id, pct) {
  const e = document.getElementById(id); if(!e) return;
  e.style.strokeDashoffset = 188 * (1 - Math.min(1, Math.max(0, pct)));
}
function setDot(id, cls) {
  const e = document.getElementById(id); if(e) e.className = 'dot ' + cls;
}
function setVal(id, text, cls) {
  const e = document.getElementById(id); if(!e) return;
  e.textContent = text;
  e.className = 'row-val ' + (cls||'');
}
function setSys(did, tid, ok) {
  const d=document.getElementById(did), t=document.getElementById(tid);
  if(d) d.className='dot '+(ok?'ok':'e');
  if(t){ t.textContent=ok?'OK':'Défaut'; t.style.color=ok?'var(--green)':'var(--red)'; }
}
function sign(v) { return v >= 0 ? '+' : ''; }

async function clearDTC() {
  if (!confirm('Effacer tous les codes défauts ?')) return;
  try {
    const r = await fetch('http://'+HOST+'/cleardtc');
    if (r.ok) {
      document.getElementById('dtc-box').innerHTML =
        '<div class="dtc-ok">✓ Codes effacés — les DEL MIL vont s\'éteindre</div>';
      setTimeout(fetchData, 2000);
    }
  } catch(e) { alert('Erreur: '+e.message); }
}

async function fetchData() {
  try {
    const r = await fetch('http://'+HOST+'/data', {cache:'no-store'});
    if (!r.ok) throw new Error('HTTP '+r.status);
    const d = await r.json();

    // Info protocole
    document.getElementById('obd-info').textContent =
      'Proto: '+d.proto+' | Baud: '+d.baud+' | OBD: '+(d.obd_ready?'✓':'✗')
      +' | Lectures: '+d.ok_reads+'/'+d.total_reads;

    // Badge connexion
    const badge = document.getElementById('badge');
    badge.className = 'badge ok';
    badge.textContent = '● Connecté';

    // ── Jauges ──
    const rpmPct  = d.nd_rpm ? 0 : d.rpm/6200;
    const spdPct  = d.nd_spd ? 0 : d.speed/178;
    const tmpPct  = d.nd_tmp ? 0 : Math.max(0, Math.min(1, d.temp/120));

    arc('g-rpm', rpmPct);
    arc('g-spd', spdPct);
    arc('g-tmp', tmpPct);

    setVal('v-rpm', d.nd_rpm ? 'N/A' : d.rpm+' tr/min', d.nd_rpm?'nd':'');
    setVal('v-spd', d.nd_spd ? 'N/A' : d.speed+' km/h', d.nd_spd?'nd':'');
    setVal('v-tmp', d.nd_tmp ? 'N/A' : d.temp+'°C',     d.nd_tmp?'nd': d.temp>=105?'alert':d.temp>=95?'warn':'');

    // Couleur jauge température selon état
    const gtmp = document.getElementById('g-tmp');
    if (gtmp) {
      if (d.temp >= 105) gtmp.style.stroke = '#ef4444';
      else if (d.temp >= 90) gtmp.style.stroke = '#22c55e';
      else gtmp.style.stroke = '#3b82f6';
    }

    // ── Paramètres moteur ──
    setVal('v-load', d.nd_load?'N/A': d.load+'%',        d.nd_load?'nd':d.load>90?'warn':'');
    setVal('v-map',  d.nd_map ?'N/A': d.map+' kPa',      d.nd_map ?'nd':'');
    setVal('v-ign',  d.nd_ign ?'N/A': d.ign+'°',         d.nd_ign ?'nd':d.ign<0?'alert':'');
    setVal('v-o2',   d.nd_o2  ?'N/A': d.o2.toFixed(3)+' V', d.nd_o2?'nd': (!d.sys_o2)?'alert':'');
    setVal('v-tps',  d.nd_tps ?'N/A': d.tps+'%',         d.nd_tps ?'nd':'');

    // ── Alimentation ──
    setVal('v-bat',  d.nd_bat ?'N/A': d.volt.toFixed(2)+' V', d.nd_bat?'nd': d.volt<11.5?'alert':d.volt<12.5?'warn':'');
    setVal('v-stft', d.nd_stft?'N/A': sign(d.stft)+d.stft.toFixed(1)+'%', d.nd_stft?'nd': Math.abs(d.stft)>10?'alert':Math.abs(d.stft)>5?'warn':'');
    setVal('v-ltft', d.nd_ltft?'N/A': sign(d.ltft)+d.ltft.toFixed(1)+'%', d.nd_ltft?'nd': Math.abs(d.ltft)>10?'alert':Math.abs(d.ltft)>5?'warn':'');

    // Dots
    setDot('d-load', d.nd_load?'nd': d.load>90?'w':'ok');
    setDot('d-map',  d.nd_map ?'nd':'ok');
    setDot('d-ign',  d.nd_ign ?'nd': d.ign<0?'e':'ok');
    setDot('d-o2',   d.nd_o2  ?'nd': !d.sys_o2?'e':'ok');
    setDot('d-tps',  d.nd_tps ?'nd':'ok');
    setDot('d-bat',  d.nd_bat ?'nd': d.volt<11.5?'e':d.volt<12.5?'w':'ok');
    setDot('d-stft', d.nd_stft?'nd': Math.abs(d.stft)>10?'e':Math.abs(d.stft)>5?'w':'ok');
    setDot('d-ltft', d.nd_ltft?'nd': Math.abs(d.ltft)>10?'e':Math.abs(d.ltft)>5?'w':'ok');

    // ── Freeze Frame ──
    document.getElementById('ff-r').textContent = d.ff_rpm  > 0 ? d.ff_rpm+' tr/m'  : '—';
    document.getElementById('ff-s').textContent = d.ff_spd  > 0 ? d.ff_spd+' km/h'  : '—';
    document.getElementById('ff-t').textContent = d.ff_tmp  !== 0 ? d.ff_tmp+'°C'   : '—';
    document.getElementById('ff-l').textContent = d.ff_load > 0 ? d.ff_load+'%'     : '—';

    // ── Systèmes ──
    setSys('s-mis','t-mis', d.sys_mis);
    setSys('s-cat','t-cat', d.sys_cat);
    setSys('s-o2', 't-o2',  d.sys_o2);
    setSys('s-egr','t-egr', d.sys_egr);

    // ── DTC ──
    const db = document.getElementById('dtc-box');
    if (d.dtc && d.dtc.length > 0) {
      db.innerHTML = d.dtc.map(c =>
        `<div class="dtc-item">
           <div class="dtc-code">⚠ ${c.code}</div>
           <div class="dtc-desc">${c.desc}</div>
         </div>`
      ).join('');
    } else {
      db.innerHTML = '<div class="dtc-ok">✓ Aucun code défaut actif</div>';
    }

    // ── Alertes NO DATA ──
    const ndList = [];
    if(d.nd_rpm) ndList.push('RPM');  if(d.nd_spd) ndList.push('Vitesse');
    if(d.nd_tmp) ndList.push('Temp'); if(d.nd_bat) ndList.push('Batterie');
    const ab = document.getElementById('alert-box');
    if (ndList.length > 0) {
      ab.style.display = 'block';
      ab.textContent = '⚠ NO DATA sur : '+ndList.join(', ')+' — voir moniteur série 115200 bps';
    } else {
      ab.style.display = 'none';
    }

    // ── Footer ──
    document.getElementById('footer').textContent =
      'MAJ '+new Date().toLocaleTimeString()+
      ' | RPM='+d.rpm+' SPD='+d.speed+
      ' TMP='+d.temp+'°C BAT='+d.volt.toFixed(2)+'V'+
      ' STFT='+sign(d.stft)+d.stft.toFixed(1)+'% LTFT='+sign(d.ltft)+d.ltft.toFixed(1)+'%';

  } catch(e) {
    const badge = document.getElementById('badge');
    badge.className = 'badge err';
    badge.textContent = '● Déconnecté';
    document.getElementById('footer').textContent = 'Erreur: '+e.message+' — Vérifier IP ESP32';
  }
}

fetchData();
setInterval(fetchData, 2500);
</script>
</body>
</html>
)HTMLEOF";

// ─────────────────────────────────────────────────────────────
//  HANDLER /data — JSON complet
// ─────────────────────────────────────────────────────────────

void handleData() {
  server.sendHeader("Access-Control-Allow-Origin", "*");
  server.sendHeader("Cache-Control", "no-cache, no-store");

  // Construire tableau DTC avec descriptions
  String dtcArray = "[";
  if (dtcJson.length() > 2) {
    String tmp = dtcJson.substring(1, dtcJson.length()-1);
    bool first = true;
    while (tmp.length() > 2) {
      int q1 = tmp.indexOf('"');
      if (q1 < 0) break;
      int q2 = tmp.indexOf('"', q1+1);
      if (q2 < 0) break;
      String code = tmp.substring(q1+1, q2);
      tmp = (tmp.length() > q2+2) ? tmp.substring(q2+2) : "";
      if (code.length() > 0) {
        if (!first) dtcArray += ",";
        dtcArray += "{\"code\":\"" + code + "\","
                    "\"desc\":\""  + dtcDescription(code) + "\"}";
        first = false;
      }
    }
  }
  dtcArray += "]";

  int okReads = totalReads - failReads;

  String json = "{";
  json += "\"rpm\":"         + String(rpm)                       + ",";
  json += "\"speed\":"       + String(vehicleSpeed)              + ",";
  json += "\"temp\":"        + String(coolantTemp)               + ",";
  json += "\"load\":"        + String(engineLoad)                + ",";
  json += "\"map\":"         + String(map_kpa)                   + ",";
  json += "\"ign\":"         + String(ignTiming)                 + ",";
  json += "\"tps\":"         + String(throttlePos)               + ",";
  json += "\"stft\":"        + String(stft, 2)                   + ",";
  json += "\"ltft\":"        + String(ltft, 2)                   + ",";
  json += "\"o2\":"          + String(o2voltage, 4)              + ",";
  json += "\"volt\":"        + String(battVoltage, 3)            + ",";
  json += "\"ff_rpm\":"      + String(ff_rpm)                    + ",";
  json += "\"ff_spd\":"      + String(ff_speed)                  + ",";
  json += "\"ff_tmp\":"      + String(ff_temp)                   + ",";
  json += "\"ff_load\":"     + String(ff_load)                   + ",";
  json += "\"sys_mis\":"     + String(misfire_ok ? "true":"false") + ",";
  json += "\"sys_cat\":"     + String(catalyst_ok? "true":"false") + ",";
  json += "\"sys_o2\":"      + String(o2_ok      ? "true":"false") + ",";
  json += "\"sys_egr\":"     + String(egr_ok     ? "true":"false") + ",";
  json += "\"obd_ready\":"   + String(obdReady   ? "true":"false") + ",";
  json += "\"baud\":"        + String(obdBaud)                   + ",";
  json += "\"total_reads\":" + String(totalReads)                + ",";
  json += "\"ok_reads\":"    + String(okReads)                   + ",";
  json += "\"nd_rpm\":"      + String(nd_rpm  ? "true":"false")  + ",";
  json += "\"nd_spd\":"      + String(nd_spd  ? "true":"false")  + ",";
  json += "\"nd_tmp\":"      + String(nd_tmp  ? "true":"false")  + ",";
  json += "\"nd_load\":"     + String(nd_load ? "true":"false")  + ",";
  json += "\"nd_tps\":"      + String(nd_tps  ? "true":"false")  + ",";
  json += "\"nd_map\":"      + String(nd_map  ? "true":"false")  + ",";
  json += "\"nd_ign\":"      + String(nd_ign  ? "true":"false")  + ",";
  json += "\"nd_stft\":"     + String(nd_stft ? "true":"false")  + ",";
  json += "\"nd_ltft\":"     + String(nd_ltft ? "true":"false")  + ",";
  json += "\"nd_o2\":"       + String(nd_o2   ? "true":"false")  + ",";
  json += "\"nd_bat\":"      + String(nd_bat  ? "true":"false")  + ",";
  json += "\"proto\":\""     + obdProtocol + "\","                ;
  json += "\"dtc\":"         + dtcArray                          ;
  json += "}";

  server.send(200, "application/json", json);
}

void handleRoot() {
  server.sendHeader("Access-Control-Allow-Origin", "*");
  server.send_P(200, "text/html", DASHBOARD_HTML);
}

void handleClearDTC() {
  server.sendHeader("Access-Control-Allow-Origin", "*");
  Serial.println("\n[DTC] ── Effacement DTC (mode 04) ──");
  String r = sendOBD("04", 4000);
  Serial.println("[DTC] Réponse effacement: '" + r + "'");
  bool ok = (r.indexOf("44") >= 0 || r.indexOf("OK") >= 0);
  Serial.println("[DTC] " + String(ok ? "✓ Codes effacés" : "? Réponse inattendue"));
  dtcJson = "[]";
  dtcRaw  = "";
  catalyst_ok = true; o2_ok = true; egr_ok = true; misfire_ok = true;
  server.send(200, "text/plain", ok ? "OK" : "SENT");
}

// ─────────────────────────────────────────────────────────────
//  ENVOI SUPABASE
// ─────────────────────────────────────────────────────────────

void pushToSupabase() {
  if (WiFi.status() != WL_CONNECTED || !obdReady) return;
  Serial.println("\n[PUSH] ── Envoi Supabase ──");

  String body = "{\"p_secret\":\"" + String(DEVICE_SECRET) + "\","
                "\"p_data\":{";
  body += "\"rpm\":"    + String(rpm)            + ",";
  body += "\"speed\":"  + String(vehicleSpeed)   + ",";
  body += "\"temp\":"   + String(coolantTemp)    + ",";
  body += "\"volt\":"   + String(battVoltage,2)  + ",";
  body += "\"load\":"   + String(engineLoad)     + ",";
  body += "\"stft\":"   + String(stft,1)         + ",";
  body += "\"ltft\":"   + String(ltft,1)         + ",";
  body += "\"o2\":"     + String(o2voltage,3)    + ",";
  body += "\"ign\":"    + String(ignTiming)      + ",";
  body += "\"map\":"    + String(map_kpa)        + ",";
  body += "\"dtc\":"    + dtcJson;
  body += "}}";

  HTTPClient http;
  http.begin(String(SUPABASE_URL) + "/rest/v1/rpc/push_obd_data");
  http.addHeader("apikey",       SUPABASE_ANON);
  http.addHeader("Content-Type", "application/json");
  http.setTimeout(8000);
  int code = http.POST(body);
  Serial.println("[PUSH] HTTP " + String(code) + (code==200?" ✓":" ✗ (vérifier Supabase)"));
  http.end();
}

// ─────────────────────────────────────────────────────────────
//  SETUP
// ─────────────────────────────────────────────────────────────

void setup() {
  Serial.begin(115200);
  delay(1000);

  Serial.println("\n╔══════════════════════════════════════════╗");
  Serial.println("║  Opel Astra H 1.4 MPI Z14XEP — v3.0     ║");
  Serial.println("║  ESP32 + SparkFun OBD-II UART (ELM327)   ║");
  Serial.println("║  Moniteur série : 115200 bps              ║");
  Serial.println("╚══════════════════════════════════════════╝");
  Serial.println("[INFO] Tags moniteur : [BAUD] [OBD] [PID] [DTC] [FF] [PUSH]");
  Serial.println("[INFO] VÉRIFIER : clé de contact sur ON ou moteur démarré");

  // Init UART2 (GPIO16=RX, GPIO17=TX)
  obdSerial.begin(9600, SERIAL_8N1, 16, 17);
  delay(600);

  // Init ELM327 + protocole
  initELM();

  // Lecture initiale si OBD OK
  if (obdReady) {
    readLiveData();
    readDTC();
    readFreezeFrame();
  }

  // Connexion WiFi
  WiFi.mode(WIFI_STA);
  WiFi.disconnect(true);
  delay(300);
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);

  Serial.print("\n[WIFI] Connexion à '" + String(WIFI_SSID) + "'");
  int tries = 0;
  while (WiFi.status() != WL_CONNECTED && tries < 40) {
    delay(500); Serial.print("."); tries++;
    yield();
  }

  if (WiFi.status() == WL_CONNECTED) {
    Serial.println("\n[WIFI] ✓ Connecté !");
    Serial.println("[WIFI] ► http://" + WiFi.localIP().toString());
    Serial.println("[WIFI]   Ouvrir cette URL dans votre navigateur");
  } else {
    Serial.println("\n[WIFI] ✗ Échec connexion WiFi");
    Serial.println("[WIFI]   SSID: " + String(WIFI_SSID));
    Serial.println("[WIFI]   Vérifier hotspot activé → redémarrage...");
    delay(3000);
    ESP.restart();
  }

  // Routes HTTP
  server.on("/",         HTTP_GET, handleRoot);
  server.on("/data",     HTTP_GET, handleData);
  server.on("/cleardtc", HTTP_GET, handleClearDTC);
  server.begin();

  Serial.println("[HTTP] Serveur prêt → http://" + WiFi.localIP().toString());
  Serial.println("════════════════════════════════════════════\n");
}

// ─────────────────────────────────────────────────────────────
//  LOOP
// ─────────────────────────────────────────────────────────────

void loop() {
  server.handleClient();

  // Lecture OBD en continu toutes les LIVE_READ_INTERVAL ms
  // (même sans navigateur connecté, les données sont fraîches)
  unsigned long now = millis();
  if (obdReady && (now - lastLiveRead >= LIVE_READ_INTERVAL)) {
    lastLiveRead = now;
    readLiveData();
  }

  // Envoi Supabase toutes les 30 secondes
  if (now - lastPush >= PUSH_INTERVAL_MS) {
    lastPush = now;
    pushToSupabase();
  }

  yield();
  delay(5);
}

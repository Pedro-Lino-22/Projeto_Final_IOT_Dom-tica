#include <ESP8266WiFi.h>
#include <PubSubClient.h>
#include <DHT.h>
#include <Wire.h>
#include <Adafruit_VEML7700.h>
#include <Servo.h>
#include <ArduinoJson.h>

// Configurações WiFi
const char* ssid = "Player_23_2";
const char* password = "1234567890";

// Configurações MQTT
const char* mqtt_server = "broker.emqx.io";
const char* topic_sensores = "IPB/IoT/Projeto/a48168a43948/Ambiente";
const char* topic_comandos = "IPB/IoT/Projeto/a48168a43948/climatizacao";
const char* topic_confirmacao = "IPB/IoT/Projeto/a48168a43948/climatizacao/confirm";
const char* topic_qualidade_ar = "IPB/IoT/Lab/AirQuality";

// Pinos dos sensores DHT22
#define DHT_FORA_PIN D3
#define DHT_DENTRO_PIN D4
#define DHT_TYPE DHT22

// esp8266 pin connected to AO pin of the MQ2 sensor
#define AO_PIN A0 

// Pinos do LED RGB
#define LED_VERMELHO D0
#define LED_VERDE D1
#define LED_AZUL D2

// Pinos dos servos
#define SERVO_CORTINA_PIN D7
#define SERVO_JANELA_PIN D8

// Pinos I2C para VEML7700
#define SDA_PIN D5
#define SCL_PIN D6

// Componentes
DHT dht_fora(DHT_FORA_PIN, DHT_TYPE);
DHT dht_dentro(DHT_DENTRO_PIN, DHT_TYPE);
Adafruit_VEML7700 veml;
Servo servo_cortina;
Servo servo_janela;

WiFiClient espClient;
PubSubClient client(espClient);

// Variáveis de estado
float T_fora, H_fora;
float T_dentro, H_dentro;
float Luz;
float Gas;
float qualidade_ar = 0.0;
int condicionador_ar = 4;
int cortina = 0;
int janela = 0;

// Limite para concentração de CO2 considerado elevado (em ppm)
const float CO2_LIMITE = 1000.0;

// Temporizadores
unsigned long lastSensorRead = 0;
const long sensorInterval = 10000; // 10 segundos
unsigned long lastMqttReconnect = 0;
const long mqttReconnectInterval = 5000; // 5 segundos

void setup_wifi() {
  WiFi.setAutoReconnect(true);
  WiFi.persistent(true);
  WiFi.mode(WIFI_STA);
  
  Serial.println();
  Serial.print("Conectando ao WiFi: ");
  Serial.println(ssid);

  WiFi.begin(ssid, password);

  unsigned long start = millis();
  while (WiFi.status() != WL_CONNECTED && millis() - start < 10000) {
    delay(500);
    Serial.print(".");
  }

  if (WiFi.status() == WL_CONNECTED) {
    Serial.println("\nWiFi conectado!");
    Serial.print("Endereço IP: ");
    Serial.println(WiFi.localIP());
  }
}

void atualizarLED() {
  digitalWrite(LED_VERMELHO, LOW);
  digitalWrite(LED_VERDE, LOW);
  digitalWrite(LED_AZUL, LOW);

  switch (condicionador_ar) {
    case 1: // Aquecendo (Laranja)
      digitalWrite(LED_VERMELHO, HIGH);
      digitalWrite(LED_VERDE, HIGH);
      break;
    case 2: // Arrefecendo (Azul)
      digitalWrite(LED_AZUL, HIGH);
      break;
    case 3: // Ventilando (Verde)
      digitalWrite(LED_VERDE, HIGH);
      break;
    case 4: // Desligado (Vermelho)
      digitalWrite(LED_VERMELHO, HIGH);
      break;
  }
}

void atualizarCortina() {
  switch (cortina) {
    case 0: // 0° (500μs)
      servo_cortina.writeMicroseconds(500);
      break;
    case 1: // 90° (1450μs)
      servo_cortina.writeMicroseconds(1450);
      break;
    case 2: // 180° (2400μs)
      servo_cortina.writeMicroseconds(2400);
      break;
  }
}

void atualizarJanela() {
  switch (janela) {
    case 0: // 0° (500μs)
      servo_janela.writeMicroseconds(500);
      break;
    case 1: // 90° (1450μs)
      servo_janela.writeMicroseconds(1450);
      break;
    case 2: // 180° (2400μs)
      servo_janela.writeMicroseconds(2400);
      break;
  }
}

void enviarConfirmacao() {
  StaticJsonDocument<128> doc;
  doc["condicionador_ar"] = condicionador_ar;
  doc["cortina"] = cortina;
  doc["janela"] = janela;

  char buffer[128];
  size_t n = serializeJson(doc, buffer);

  if (client.publish(topic_confirmacao, buffer, n)) {
    Serial.print("Confirmação enviada: ");
    Serial.println(buffer);
  }
}

void callback(char* topic, byte* payload, unsigned int length) {
  Serial.print("Mensagem recebida: ");
  Serial.write(payload, length);
  Serial.println();

  // Verificar se a mensagem é do tópico de qualidade do ar
  if (strcmp(topic, topic_qualidade_ar) == 0) {
    // Processar JSON para qualidade do ar
    StaticJsonDocument<256> doc;
    DeserializationError error = deserializeJson(doc, payload, length);

    if (error) {
      Serial.print("Erro no JSON: ");
      Serial.println(error.c_str());
      return;
    }

    // Extrair o valor da concentração de CO2
    if (doc.containsKey("co2_eqv")) {
      qualidade_ar = doc["co2_eqv"];
      Serial.print("Concentração de CO2: ");
      Serial.println(qualidade_ar);
    }
  } else {
    // Processar JSON para os outros comandos
    StaticJsonDocument<256> doc;
    DeserializationError error = deserializeJson(doc, payload, length);

    if (error) {
      Serial.print("Erro no JSON: ");
      Serial.println(error.c_str());
      return;
    }

    if (doc.containsKey("condicionador_ar")) {
      condicionador_ar = doc["condicionador_ar"];
      atualizarLED();
    }
    if (doc.containsKey("cortina")) {
      cortina = doc["cortina"];
      atualizarCortina();
    }
    if (doc.containsKey("janela")) {
      janela = doc["janela"];
      atualizarJanela();
    }

    enviarConfirmacao();
  }
}

void reconnect() {
  if (!client.connected() && millis() - lastMqttReconnect > mqttReconnectInterval) {
    Serial.print("Conectando ao MQTT...");
    
    String clientId = "ESP8266Client-";
    clientId += String(random(0xffff), HEX);

    if (client.connect(clientId.c_str())) {
      Serial.println("Conectado!");
      client.subscribe(topic_comandos);
      client.subscribe(topic_qualidade_ar);
      lastMqttReconnect = 0;
    } else {
      Serial.print("Falha, rc=");
      Serial.print(client.state());
      lastMqttReconnect = millis();
    }
  }
}

void lerSensores() {
  T_fora = dht_fora.readTemperature();
  H_fora = dht_fora.readHumidity();
  T_dentro = dht_dentro.readTemperature();
  H_dentro = dht_dentro.readHumidity();
  Luz = veml.readLux();

  // Verificar valores inválidos
  if (isnan(T_fora) || isnan(H_fora)) {
    Serial.println("Erro no DHT externo");
    T_fora = H_fora = NAN;
  }
  if (isnan(T_dentro) || isnan(H_dentro)) {
    Serial.println("Erro no DHT interno");
    T_dentro = H_dentro = NAN;
  }
}

void publicarSensores() {
  StaticJsonDocument<256> doc;
  doc["T_fora"] = T_fora;
  doc["H_fora"] = H_fora;
  doc["T_dentro"] = T_dentro;
  doc["H_dentro"] = H_dentro;
  doc["Luz"] = Luz;
  doc["qualidade_ar"] = qualidade_ar;

  char buffer[256];
  size_t n = serializeJson(doc, buffer);

  if (client.publish(topic_sensores, buffer, n)) {
    Serial.print("Dados publicados: ");
    Serial.println(buffer);
  }
}

void setup() {
  Serial.begin(115200);
  
  // Inicializar pinos
  pinMode(LED_VERMELHO, OUTPUT);
  pinMode(LED_VERDE, OUTPUT);
  pinMode(LED_AZUL, OUTPUT);
  
  dht_fora.begin();
  dht_dentro.begin();
  
  Wire.begin(SDA_PIN, SCL_PIN);
  if (!veml.begin()) {
    Serial.println("Erro no VEML7700!");
    while(1);
  }
  
  servo_cortina.attach(SERVO_CORTINA_PIN, 500, 2400);
  servo_janela.attach(SERVO_JANELA_PIN, 500, 2400);
  
  atualizarLED();
  atualizarCortina();
  atualizarJanela();
  
  setup_wifi();
  
  client.setServer(mqtt_server, 1883);
  client.setCallback(callback);
  client.setBufferSize(512);
  
  client.subscribe(topic_comandos);
  client.subscribe(topic_qualidade_ar);
}

void loop() {
  if (WiFi.status() != WL_CONNECTED) {
    setup_wifi();
  }
  
  if (!client.connected()) {
    reconnect();
  }
  client.loop();
  
  if (millis() - lastSensorRead >= sensorInterval) {
    lerSensores();
    publicarSensores();
    lastSensorRead = millis();
  }
  
  if (qualidade_ar > CO2_LIMITE) {
    bool changed = false;
    if (janela != 1) {
      janela = 1;
      atualizarJanela();
      changed = true;
    }
    if (cortina != 1) {
      cortina = 1;
      atualizarCortina();
      changed = true;
    }
    if (changed) {
      Serial.println("CO2 alto: Ajustando janela e cortina automaticamente");
      enviarConfirmacao();
    }
  }
  
  yield();
}

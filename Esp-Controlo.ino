#include <Wire.h>
#include <PN532_I2C.h>
#include <PN532.h>
#include <ESP8266WiFi.h>
#include <PubSubClient.h>
#include <ArduinoJson.h>

// Configurações de rede
const char* ssid = "Player_23_2";
const char* password = "1234567890";
const char* mqtt_server = "broker.emqx.io";

// Definição dos pinos
#define SDA_PIN D2
#define SCL_PIN D1
const int portaPin = D0;
const int buzzerPin = D5;
const int led1Pin = D7;
const int led2Pin = D6;
const int button1Pin = D3;
const int button2Pin = D4;

// Instâncias dos objetos
PN532_I2C pn532i2c(Wire);
PN532 nfc(pn532i2c);
WiFiClient espClient;
PubSubClient client(espClient);

// Variáveis globais
DynamicJsonDocument tags_permitidas(1024);
String cartao_lido = "";
bool modo_adicionar = false;
bool modo_remover = false;
bool primeira_tag_adicionada = false;
bool porta_ativa = false;
unsigned long tempo_porta = 0;
const unsigned long tempo_porta_aberta = 2000;
unsigned long previousReadMillis = 0;
const unsigned long readInterval = 500;  // 1 segundos
bool led1State = false;
bool led2State = false;

void setup_wifi() {
  delay(10);
  Serial.println("\nConectando a: " + String(ssid));
  WiFi.begin(ssid, password);

  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println("\nConectado com IP: " + WiFi.localIP().toString());
}

void emitir_som(int repeticoes) {
  for (int i = 0; i < repeticoes; i++) {
    tone(buzzerPin, 1000, 200);
    delay(300);
    noTone(buzzerPin);
  }
}

bool tag_autorizada(String tag) {
  JsonArray array = tags_permitidas.as<JsonArray>();
  for (JsonVariant valor : array) {
    if (valor.as<String>() == tag) return true;
  }
  return false;
}

void adicionar_tag(String tag) {
  JsonArray array = tags_permitidas.as<JsonArray>();
  if (!tag_autorizada(tag)) {
    array.add(tag);
    Serial.println("Tag adicionada: " + tag);
    client.publish("IPB/IoT/Projeto/a48168a43948/tags_status", ("Adicionada: " + tag).c_str());
  }
}

void remover_tag(String tag) {
  JsonArray array = tags_permitidas.as<JsonArray>();
  for (size_t i = 0; i < array.size(); i++) {
    if (array[i].as<String>() == tag) {
      array.remove(i);
      Serial.println("Tag removida: " + tag);
      client.publish("IPB/IoT/Projeto/a48168a43948/tags_status", ("Removida: " + tag).c_str());

      if (array.size() == 0) {
        primeira_tag_adicionada = false;
        Serial.println("Lista de tags reiniciada");
      }
      break;
    }
  }
}

void callback(String topic, byte* payload, unsigned int length) {
  Serial.print("Message arrived [");
  Serial.print(topic);
  Serial.print("] ");
  String strPayload = "";

  for (int i = 0; i < length; i++) {
    strPayload += (char)payload[i];
  }

  int comando = strPayload.toInt();

  if (topic == "IPB/IoT/Projeto/a48168a43948/tags") {
    if (comando == 0) {
      modo_adicionar = false;
      modo_remover = false;
      // Limpar tags se desejado quando volta para modo normal? Comente se não quiser limpar:
      // limpar_tags();

      Serial.println("Modo normal ativado via MQTT");
      client.publish("IPB/IoT/Projeto/a48168a43948/tags/confirm", "0");
      client.publish("IPB/IoT/Projeto/a48168a43948/status", "Modo normal ativado");
    } else if (comando == 1) {
      modo_adicionar = true;
      modo_remover = false;
      Serial.println("Modo adicionar ativado via MQTT");
      client.publish("IPB/IoT/Projeto/a48168a43948/tags/confirm", "1");
      client.publish("IPB/IoT/Projeto/a48168a43948/status", "Modo adicionar ativado");
    } else if (comando == 2) {
      modo_remover =IPB/IoT/Projeto/a48168a43948/status true;
      modo_adicionar = false;
      Serial.println("Modo remover ativado via MQTT");
      client.publish("", "Modo remover ativado");
    }

    // Você pode incluir aqui ações adicionais ao mudar de modo,
    // por exemplo, acionar um LED indicando modo ativo, emitir som, etc.
  }

  if (topic == "IPB/IoT/Projeto/a48168a43948/Luzes/1") {
    if (comando == 1) {
      digitalWrite(led1Pin, HIGH);  // Liga o rele 1
      led1State = true;
      client.publish("IPB/IoT/Projeto/a48168a43948/Luzes/1/estado", "1");
    } else if (comando == 0) {
      digitalWrite(led1Pin, LOW);  // Desliga o rele 1
      led1State = false;
      client.publish("IPB/IoT/Projeto/a48168a43948/Luzes/1/estado", "0");
    }
  } else if (topic == "IPB/IoT/Projeto/a48168a43948/Luzes/2") {
    if (comando == 1) {
      digitalWrite(led2Pin, HIGH);  // Liga o rele 2
      led2State = true;
      client.publish("IPB/IoT/Projeto/a48168a43948/Luzes/2/estado", "1");
    } else if (comando == 0) {
      digitalWrite(led2Pin, LOW);  // Desliga o rele 2
      led2State = false;
      client.publish("IPB/IoT/Projeto/a48168a43948/Luzes/2/estado", "0");
    }
  }
}

void reconnect() {
  while (!client.connected()) {
    Serial.print("Conectando ao MQTT...");
    String clientId = "NFC-Controller-";
    clientId += String(random(0xffff), HEX);

    if (client.connect(clientId.c_str())) {
      Serial.println("Conectado!");
      client.subscribe("IPB/IoT/Projeto/a48168a43948/tags");
      client.subscribe("IPB/IoT/Projeto/a48168a43948/Luzes/1");
      client.subscribe("IPB/IoT/Projeto/a48168a43948/Luzes/2");
    } else {
      Serial.print("Falhou, rc=");
      Serial.print(client.state());
      Serial.println(" Tentando novamente em 5s...");
      delay(5000);
    }
  }
}

void processar_tag() {
  if (!primeira_tag_adicionada) {
    adicionar_tag(cartao_lido);
    primeira_tag_adicionada = true;
    emitir_som(2);
    return;
  }

  if (modo_adicionar) {
    adicionar_tag(cartao_lido);
    emitir_som(1);
    return;
  }

  if (modo_remover) {
    remover_tag(cartao_lido);
    emitir_som(3);  // Som diferente para remoção
    return;
  }

  if (tag_autorizada(cartao_lido)) {
    digitalWrite(portaPin, LOW);
    client.publish("IPB/IoT/Projeto/a48168a43948/porta", "1");
    porta_ativa = true;
    tempo_porta = millis();
    emitir_som(1);
    Serial.println("Acesso autorizado");
  } else {
    emitir_som(4);
    Serial.println("Acesso negado");
  }
}

void setup() {
  Serial.begin(115200);

  pinMode(portaPin, OUTPUT);
  pinMode(buzzerPin, OUTPUT);
  digitalWrite(portaPin, HIGH);
  pinMode(led1Pin, OUTPUT);
  pinMode(led2Pin, OUTPUT);
  pinMode(button1Pin, INPUT_PULLUP);
  pinMode(button2Pin, INPUT_PULLUP);

  digitalWrite(led1Pin, HIGH);  // Inicializa rele 1 desligado
  digitalWrite(led2Pin, HIGH);  // Inicializa rele 2 desligado

  Wire.begin(SDA_PIN, SCL_PIN);
  nfc.begin();

  uint32_t versiondata = nfc.getFirmwareVersion();
  if (!versiondata) {
    Serial.println("PN532 não detectado!");
    while (1)
      ;
  }
  nfc.SAMConfig();
  Serial.println("PN532 pronto!");

  setup_wifi();
  client.setServer(mqtt_server, 1883);
  client.setCallback(callback);

  tags_permitidas.to<JsonArray>();
}

void verificarTagsNFC() {
    unsigned long currentMillis = millis();
    if (currentMillis - previousReadMillis >= readInterval) {
        previousReadMillis = currentMillis;
        
        uint8_t uid[7];
        uint8_t uidLength;
        
        // Leitura com timeout de 50ms
        if (nfc.readPassiveTargetID(PN532_MIFARE_ISO14443A, uid, &uidLength, 50)) {
            cartao_lido = "";
            for (uint8_t i = 0; i < uidLength; i++) {
                if (uid[i] < 0x10) cartao_lido += "0";
                cartao_lido += String(uid[i], HEX);
                if (i < uidLength - 1) cartao_lido += " ";
            }
            cartao_lido.toUpperCase();
            processar_tag();
        }
    }
}

void loop() {
  while (WiFi.status() != WL_CONNECTED) {
    WiFi.reconnect();
  }
  if (!client.connected()) {
    Serial.println("Node disconnected from Broker. Trying to connect.. ");
    reconnect();
  }
  client.loop();

  // Leitura dos botões
  if (digitalRead(button1Pin) == LOW) {  // Botão 1 pressionado
    led1State = !led1State;              // Alterna o estado do LED 1
    digitalWrite(led1Pin, led1State ? HIGH : LOW);
    client.publish("IPB/IoT/Projeto/a48168a43948/Luzes/1/estado", led1State ? "1" : "0");
    delay(200);  // Debounce
  }

  if (digitalRead(button2Pin) == LOW) {  // Botão 2 pressionado
    led2State = !led2State;              // Alterna o estado do LED 2
    digitalWrite(led2Pin, led2State ? HIGH : LOW);
    client.publish("IPB/IoT/Projeto/a48168a43948/Luzes/2/estado", led2State ? "1" : "0");
    delay(200);  // Debounce
  }

  //porta
  if (porta_ativa && (millis() - tempo_porta >= tempo_porta_aberta)) {
    digitalWrite(portaPin, HIGH);
    client.publish("IPB/IoT/Projeto/a48168a43948/porta", "0");
    porta_ativa = false;
    Serial.println("Porta fechada");
  }

  client.loop();
  verificarTagsNFC();

}
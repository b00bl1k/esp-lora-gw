/**
 * MIT License
 *
 * Copyright (c) 2022 Alexey Ryabov
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */

#include <ESP8266WiFi.h>
#include <WiFiUdp.h>
#include <SPI.h>
#include <LoRa.h>
#include <ArduinoLog.h>
#include <ArduinoJson.h>
#include <base64.hpp>
#include <NTPClient.h>
#include <TimeLib.h>

#define FREQUENCY 868900000
#define SPREAD_FACTOR 12
#define CODING_RATE4 5
#define GPS_LAT 55
#define GPS_LNG 37
#define GPS_ALT 7
#define TIMEZONE_SEC (3600 * 3) // sec
#define TIMEZONE_STR "MSK"
#define NTP_SERVER "ru.pool.ntp.org"
#define PULL_DATA_INTERVAL 120000 // ms
#define PUSH_STAT_DATA_INTERVAL 30000

#define LORA_PUBLIC_SYNCWORD 0x34
#define PROTOCOL_VERSION 0x02
#define PKT_PUSH_DATA 0x00
#define PKT_PUSH_ACK 0x01
#define PKT_PULL_DATA 0x02
#define PKT_PULL_RESP 0x03
#define PKT_PULL_ACK 0x04
#define PKT_TX_ACK 0x05

struct UplinkData
{
    bool rx;
    uint32_t tmst;
    int rssi;
    float snr;
    uint8_t packet[255];
    uint8_t packetSize;
};

struct DownlinkData
{
    bool tx;
    bool sent;
    uint32_t tmst;
    uint8_t token[2];
    uint8_t packet[255];
    uint8_t packetSize;
};

struct StatData
{
    uint32_t rxnb; // Number of radio packets received
    uint32_t rxok; // Number of radio packets received with a valid PHY CRC
    uint32_t rxfw; // Number of radio packets forwarded
    uint32_t upnb; // Number of upstream datagrams
    uint32_t upok; // Number of upstream datagrams that were acknowledged
    uint32_t dwnb; // Number of downlink datagrams received
    uint32_t txnb; // Number of packets emitted
};

const int PinCs = 16;
const int PinReset = 0;
const int PinIrq = 15;
const char * WifiSsid = "SSID";
const char * WifiPassword = "PASSWORD";
unsigned int ServerPort = 1700;
const char * ServerHost = "eu1.cloud.thethings.network";
WiFiUDP Udp;
WiFiUDP NtpUDP;
StaticJsonDocument<312> JsonDoc;
NTPClient TimeClient(NtpUDP, NTP_SERVER, TIMEZONE_SEC, 60000 * 5);
struct UplinkData Uplink;
struct DownlinkData Downlink;
struct StatData Stat;
uint8_t UdpBuffer[1024];
byte MacAddr[6];
uint32_t LastPullDataTime = -PULL_DATA_INTERVAL;
uint32_t LastPushStatDataTime;
uint8_t LastPullToken[2];
uint8_t LastPushDataToken[2];

void OnLoraReceive(int packetSize)
{
    Uplink.tmst = micros();
    Uplink.rssi = LoRa.packetRssi();
    Uplink.snr = LoRa.packetSnr();
    Uplink.packetSize = packetSize;

    for (int i = 0; i < packetSize; i++)
    {
        Uplink.packet[i] = LoRa.read();
    }

    Uplink.rx = true;
}

void SendUdp(const uint8_t * data, int len)
{
    int written;

    if (WiFi.status() != WL_CONNECTED)
    {
        Log.error("SendUdp: no network" CR);
        return;
    }

    Udp.beginPacket(ServerHost, ServerPort);
    written = Udp.write(data, len);
    if (written != len)
    {
        Log.error("SendUdp: Error write" CR);
    }

    Udp.endPacket();
}

void RecvUdp()
{
    JsonObject root;
    DeserializationError err;
    int udpPacketSize;
    const char * data;

    udpPacketSize = Udp.parsePacket();
    if (udpPacketSize == 0)
    {
        return;
    }

    if (udpPacketSize > sizeof(UdpBuffer))
    {
        Log.warning("Received UDP packet is too large" CR);
        udpPacketSize = sizeof(UdpBuffer);
    }

    Udp.read(UdpBuffer, udpPacketSize);

    if (udpPacketSize < 4)
    {
        Log.warning("Received UDP packet is too small" CR);
        return;
    }

    if (UdpBuffer[0] != PROTOCOL_VERSION)
    {
        Log.warning("Invalid protocol version %d" CR, UdpBuffer[0]);
        return;
    }

    switch (UdpBuffer[3])
    {
    case PKT_PULL_ACK:
        Log.verbose("Recv PULL_ACK" CR);
        if ((UdpBuffer[1] != LastPullToken[0]) ||
            (UdpBuffer[2] != LastPullToken[1]))
        {
            Log.warning("Invalid token" CR);
        }
        else
        {
            Stat.upok++;
        }
        break;

    case PKT_PULL_RESP:
        Log.verbose("Recv PULL_RESP" CR);
        UdpBuffer[udpPacketSize] = '\n';
        UdpBuffer[udpPacketSize + 1] = '\0';
        Log.verbose((char *)UdpBuffer + 4);
        err = deserializeJson(JsonDoc, (char *)UdpBuffer + 4);
        if (err != DeserializationError::Ok)
        {
            Log.error("Json decode error %d" CR, err);
            break;
        }
        root = JsonDoc.as<JsonObject>();
        data = root["txpk"]["data"];
        decode_base64((uint8_t *)data, Downlink.packet);
        Downlink.packetSize = root["txpk"]["size"];
        Downlink.tmst = root["txpk"]["tmst"].as<unsigned long>();
        Downlink.token[0] = UdpBuffer[1];
        Downlink.token[1] = UdpBuffer[2];
        Downlink.tx = true;
        Stat.dwnb++;
        break;

    case PKT_PUSH_ACK:
        Log.verbose("Recv PUSH_ACK" CR);
        if ((UdpBuffer[1] != LastPushDataToken[0]) ||
            (UdpBuffer[2] != LastPushDataToken[1]))
        {
            Log.warning("Invalid token" CR);
        }
        else
        {
            Stat.upok++;
        }
        break;

    default:
        Log.warning("Recv unknown message type" CR);
    }
}

void SendPullData()
{
    int offset = 0;

    UdpBuffer[offset++] = PROTOCOL_VERSION;
    UdpBuffer[offset++] = LastPullToken[0] = (uint8_t)rand();
    UdpBuffer[offset++] = LastPullToken[1] = (uint8_t)rand();
    UdpBuffer[offset++] = PKT_PULL_DATA;
    UdpBuffer[offset++] = MacAddr[0];
    UdpBuffer[offset++] = MacAddr[1];
    UdpBuffer[offset++] = MacAddr[2];
    UdpBuffer[offset++] = 0xff;
    UdpBuffer[offset++] = 0xff;
    UdpBuffer[offset++] = MacAddr[3];
    UdpBuffer[offset++] = MacAddr[4];
    UdpBuffer[offset++] = MacAddr[5];

    SendUdp(UdpBuffer, offset);
    Stat.upnb++;
}

void SendTxAck()
{
    int offset = 0;

    UdpBuffer[offset++] = PROTOCOL_VERSION;
    UdpBuffer[offset++] = Downlink.token[0];
    UdpBuffer[offset++] = Downlink.token[1];
    UdpBuffer[offset++] = PKT_TX_ACK;
    UdpBuffer[offset++] = MacAddr[0];
    UdpBuffer[offset++] = MacAddr[1];
    UdpBuffer[offset++] = MacAddr[2];
    UdpBuffer[offset++] = 0xff;
    UdpBuffer[offset++] = 0xff;
    UdpBuffer[offset++] = MacAddr[3];
    UdpBuffer[offset++] = MacAddr[4];
    UdpBuffer[offset++] = MacAddr[5];

    SendUdp(UdpBuffer, offset);
}

void DebugLoRaWanMessage(const uint8_t * msg)
{
    char TempBuf[32];
    int offset;
    int i;
    int mtype = msg[0] >> 5;
    const char * alphas = "0123456789ABCDEF";

    switch (mtype)
    {
    case 0x0:
        for (i = 0, offset = 0; i < 8; i++)
        {
            TempBuf[offset++] = alphas[(msg[16 - i] >> 4)];
            TempBuf[offset++] = alphas[(msg[16 - i] & 0xf)];
        }
        TempBuf[offset] = '\0';
        Log.verbose("Join-Request from DevEUI: %s" CR, TempBuf);
        break;

    case 0x2:
    case 0x4:
        for (i = 0, offset = 0; i < 4; i++)
        {
            TempBuf[offset++] = alphas[(msg[4 - i] >> 4)];
            TempBuf[offset++] = alphas[(msg[4 - i] & 0xf)];
        }
        TempBuf[offset] = '\0';
        Log.verbose("Uplink from DevAddr: %s" CR, TempBuf);
        break;
    }
}

void SendPushData()
{
    int ackr = 0;
    int snr;
    int len;
    int offset = 0;
    char timestamp[32];

    if (Uplink.rx == false)
    {
        return;
    }

    Stat.rxnb++;
    Stat.rxok++;

    UdpBuffer[offset++] = PROTOCOL_VERSION;
    UdpBuffer[offset++] = LastPushDataToken[0] = (uint8_t)rand();
    UdpBuffer[offset++] = LastPushDataToken[1] = (uint8_t)rand();
    UdpBuffer[offset++] = PKT_PUSH_DATA;
    UdpBuffer[offset++] = MacAddr[0];
    UdpBuffer[offset++] = MacAddr[1];
    UdpBuffer[offset++] = MacAddr[2];
    UdpBuffer[offset++] = 0xff;
    UdpBuffer[offset++] = 0xff;
    UdpBuffer[offset++] = MacAddr[3];
    UdpBuffer[offset++] = MacAddr[4];
    UdpBuffer[offset++] = MacAddr[5];

    snr = (int)(Uplink.snr * 10);
    len = snprintf(
        (char *)&UdpBuffer[offset],
        sizeof(UdpBuffer) - offset,
        "{\"rxpk\":[{\"tmst\":%lu,\"chan\":0,\"rfch\":0,\"freq\":%d.%06d,\"stat\":1,"
        "\"modu\":\"LORA\",\"datr\":\"SF%dBW125\",\"codr\":\"4/%d\",\"rssi\":%d,"
        "\"lsnr\":%d.%d,\"size\":%d,\"data\":\"",
        Uplink.tmst,
        FREQUENCY / 1000000,
        FREQUENCY % 1000000,
        SPREAD_FACTOR,
        CODING_RATE4,
        Uplink.rssi,
        snr / 10,
        (snr < 0) ? (snr * -1) % 10 : snr % 10,
        Uplink.packetSize
    );
    offset += len;

    len = encode_base64(Uplink.packet, Uplink.packetSize, &UdpBuffer[offset]);
    offset += len;

    int lat = GPS_LAT * 100000;
    int lng = GPS_LNG * 100000;
    Log.verbose("Stat.upok=%d, Stat.upnb=%d" CR, Stat.upok, Stat.upnb);
    if (Stat.upnb > 0)
    {
        ackr = Stat.upok * 1000 / Stat.upnb;
    }

    setTime(TimeClient.getEpochTime());
    sprintf(timestamp, "%04d-%02d-%02d %02d:%02d:%02d " TIMEZONE_STR,
        year(), month(), day(), hour(), minute(), second());

    len = snprintf(
        (char *)&UdpBuffer[offset],
        sizeof(UdpBuffer) - offset,
        "\"}],\"stat\":{\"time\":\"%s\",\"lati\":%d.%05d,\"long\":%d.%05d,\"alti\":%d,"
        "\"rxnb\":%d,\"rxok\":%d,\"rxfw\":%d,\"ackr\":%d.%d,\"dwnb\":%d,\"txnb\":%d}}",
        timestamp,
        lat / 100000,
        lat % 100000,
        lng / 100000,
        lng % 100000,
        GPS_ALT,
        Stat.rxnb,
        Stat.rxok,
        Stat.rxfw,
        ackr / 10,
        ackr % 10,
        Stat.dwnb,
        Stat.txnb
    );
    offset += len;

    memset(&Stat, 0, sizeof(Stat));

    UdpBuffer[offset] = '\n';
    UdpBuffer[offset + 1] = '\0';
    Log.verbose((char *)UdpBuffer + 12);
    Uplink.rx = false;

    SendUdp(UdpBuffer, offset);

    DebugLoRaWanMessage(Uplink.packet);
    Stat.upnb++;
    Stat.rxfw++;
}

void SendStat()
{
    int ackr = 0;
    int len;
    int offset = 0;
    char timestamp[32];

    UdpBuffer[offset++] = PROTOCOL_VERSION;
    UdpBuffer[offset++] = LastPushDataToken[0] = (uint8_t)rand();
    UdpBuffer[offset++] = LastPushDataToken[1] = (uint8_t)rand();
    UdpBuffer[offset++] = PKT_PUSH_DATA;
    UdpBuffer[offset++] = MacAddr[0];
    UdpBuffer[offset++] = MacAddr[1];
    UdpBuffer[offset++] = MacAddr[2];
    UdpBuffer[offset++] = 0xff;
    UdpBuffer[offset++] = 0xff;
    UdpBuffer[offset++] = MacAddr[3];
    UdpBuffer[offset++] = MacAddr[4];
    UdpBuffer[offset++] = MacAddr[5];

    int lat = GPS_LAT * 100000;
    int lng = GPS_LNG * 100000;
    Log.verbose("Stat.upok=%d, Stat.upnb=%d" CR, Stat.upok, Stat.upnb);
    if (Stat.upnb > 0)
    {
        ackr = Stat.upok * 1000 / Stat.upnb;
    }

    setTime(TimeClient.getEpochTime());
    sprintf(timestamp, "%04d-%02d-%02d %02d:%02d:%02d " TIMEZONE_STR,
        year(), month(), day(), hour(), minute(), second());

    len = snprintf(
        (char *)&UdpBuffer[offset],
        sizeof(UdpBuffer) - offset,
        "{\"stat\":{\"time\":\"%s\",\"lati\":%d.%05d,\"long\":%d.%05d,\"alti\":%d,"
        "\"rxnb\":%d,\"rxok\":%d,\"rxfw\":%d,\"ackr\":%d.%d,\"dwnb\":%d,\"txnb\":%d}}",
        timestamp,
        lat / 100000,
        lat % 100000,
        lng / 100000,
        lng % 100000,
        GPS_ALT,
        Stat.rxnb,
        Stat.rxok,
        Stat.rxfw,
        ackr / 10,
        ackr % 10,
        Stat.dwnb,
        Stat.txnb
    );
    offset += len;

    memset(&Stat, 0, sizeof(Stat));

    UdpBuffer[offset] = '\n';
    UdpBuffer[offset + 1] = '\0';
    Log.verbose((char *)UdpBuffer + 12);

    SendUdp(UdpBuffer, offset);
    Stat.upnb++;
}

void WaitUntil(uint32_t tmst)
{
    int32_t delta = tmst - micros();

    while (delta > 16000)
    {
        delay(16);
        delta -= 16000;
    }

    if (delta > 0)
    {
        delayMicroseconds(delta);
    }
}

void SendDownlink()
{
    int diff;
    uint32_t tNow;

    if (Downlink.tx == false)
    {
        return;
    }

    if (Downlink.sent == false)
    {
        tNow = micros();
        diff = Downlink.tmst - tNow;
        if (diff > 32000)
        {
            return;
        }

        LoRa.idle();
        LoRa.enableInvertIQ();
        LoRa.beginPacket();
        LoRa.write(Downlink.packet, Downlink.packetSize);
        WaitUntil(Downlink.tmst);
        tNow = micros();
        LoRa.endPacket(true);
        Downlink.sent = true;
        Log.verbose("Sent on: ");
        Serial.println(tNow);
    }
    else
    {
        if (LoRa.beginPacket() != 0)
        {
            // tx done
            LoRa.disableInvertIQ();
            LoRa.receive();
            SendTxAck();
            Stat.txnb++;
            Downlink.sent = false;
            Downlink.tx = false;
        }
    }
}

void setup()
{
    Serial.begin(115200);
    Log.begin(LOG_LEVEL_VERBOSE, &Serial, true);
    Log.verbose(CR "Started" CR);

    LoRa.setPins(PinCs, PinReset, PinIrq);
    if (!LoRa.begin(FREQUENCY))
    {
        Log.fatal("Starting LoRa failed!" CR);
        while (1);
    }

    WiFi.macAddress(MacAddr);
    WiFi.mode(WIFI_STA);
    WiFi.begin(WifiSsid, WifiPassword);

    Log.verbose("Connect to WiFi network '%s'" CR, WifiSsid);

    while (WiFi.status() != WL_CONNECTED)
    {
        delay(500);
    }

    WiFi.setAutoConnect(true);
    WiFi.setAutoReconnect(true);

    TimeClient.begin();

    IPAddress localIp = WiFi.localIP();
    Log.verbose("Connected. IP address: %d.%d.%d.%d" CR,
        localIp[0], localIp[1], localIp[2], localIp[3]);
    Log.notice("Gateway ID: %x %x %x FF FF %x %x %x" CR,
        MacAddr[0], MacAddr[1], MacAddr[2],
        MacAddr[3], MacAddr[4], MacAddr[5]);

    if (Udp.begin(1700) == 0)
    {
        Log.error("Start UDP server failed");
    }

    LoRa.setSpreadingFactor(SPREAD_FACTOR);
    LoRa.setSignalBandwidth(125E3);
    LoRa.setCodingRate4(CODING_RATE4);
    LoRa.setSyncWord(LORA_PUBLIC_SYNCWORD);
    LoRa.setTxPower(14);
    LoRa.onReceive(OnLoraReceive);
    LoRa.disableInvertIQ();
    LoRa.receive();
    Log.verbose("Setup completed" CR);
}

void loop()
{
    uint32_t timeNow = millis();

    TimeClient.update();

    RecvUdp();
    SendPushData();
    SendDownlink();

    if ((timeNow - LastPullDataTime) > PULL_DATA_INTERVAL)
    {
        LastPullDataTime = timeNow;
        Log.verbose("Send PULL_DATA" CR);
        SendPullData();
    }

    if ((timeNow - LastPushStatDataTime) > PUSH_STAT_DATA_INTERVAL)
    {
        LastPushStatDataTime = timeNow;
        SendStat();
    }
}

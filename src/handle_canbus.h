// Tambahkan deklarasi di awal file, setelah includes
// 	coryjfowler/mcp_can@^1.5.1
#include <SPI.h>
#include <mcp_can.h>
#define SPI_SCK 2
#define SPI_MISO 15
#define SPI_MOSI 14
#define CAN_INT 4
#define CAN_CS 5
MCP_CAN CAN(CAN_CS);

String webCANId, webCANData;
bool communicationTimeout = false;
void canbusSetup()
{
    SPI.begin(SPI_SCK, SPI_MISO, SPI_MOSI);

    // Initialize CAN bus
    if (CAN.begin(MCP_ANY, CAN_250KBPS, MCP_8MHZ) != CAN_OK)
    {
        Serial.println("CAN init failed");
    }else{
        Serial.println("CAN Ready");
    }
    CAN.setMode(MCP_NORMAL);
    pinMode(CAN_INT, INPUT);

}

// Tambahkan variabel global untuk melacak waktu
unsigned long lastCanReceiveTime = 0;
const unsigned long timeoutReceived = 15000; // Contoh: 5000 ms (5 detik)

// Function to process received CAN messages
void handleReceivingCanbus()
{
    if (digitalRead(CAN_INT) == LOW)
    { // Check for CAN interrupt
        unsigned long id;
        byte len;
        byte msgData[8];

        if (CAN.readMsgBuf(&id, &len, msgData) == CAN_OK)
        {
            // Update waktu terakhir pesan CAN diterima
            lastCanReceiveTime = millis();

            // Update CAN display data
            webCANId = "0x" + String(id, HEX);
            webCANData = "";
            for (byte i = 0; i < len; i++)
            {
                if (msgData[i] < 0x10)
                    webCANData += "0";
                webCANData += String(msgData[i], HEX) + " ";
            }
            Serial.print("canId:");
            Serial.println(webCANId);
            Serial.print("canData:");
            Serial.println(webCANData);
        }
    }

    // Check for timeout
    if (millis() - lastCanReceiveTime > timeoutReceived)
    {
        communicationTimeout = true; 
    }
    else
    {
        communicationTimeout = false;
    }
}
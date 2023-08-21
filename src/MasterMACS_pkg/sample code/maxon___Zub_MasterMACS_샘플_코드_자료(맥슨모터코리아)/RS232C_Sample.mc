#include <SysDef.mh>
#include "C:\Users\mmkhych\Desktop\Zub\SDK\ApossC-SDK-V01-12\SDK\SDK_ApossC.mc"

// Parameters for the SDK function

dim long msg_R[100];
dim long msg_S[100];





#pragma SmConfig {1,
                  25,
                  6,
                  1,
                  12,
                  0,
                  2}



SmEvent TickPeriod {PRM_CLOCK}
SmEvent MsgReceived  { }
SmEvent MsgSend { }


SmState Root
    {
    SIG_INIT      = {
                    SmSystem(SERIALRECV, id, MsgReceived);     // Post only to this machine when a message is received.


                    Sysvar[0x01220300 + 10] = 0;            // No parity.
                    Sysvar[0x01220300 + 11] = 115200;       // Baudrate.

                    SerialInit(0x0002);                        // Initialize the DCP interface (RS485/RS232 with EOT detection).

                    }

    MsgReceived   = {
                    SerialReadComm(msg_R);
                    print("Message received: ",radixstr(msg_R[0],16)," ",radixstr(msg_R[1],16)," ",radixstr(msg_R[2],16)," ",radixstr(msg_R[3],16)," ",radixstr(msg_R[4],16)," ",radixstr(msg_R[5],16)," ",radixstr(msg_R[6],16)," ",radixstr(msg_R[7],16));
                    }


    }
SmState Root1
    {
    SIG_INIT = {
                      Sysvar[0x01220300 + 10] = 0;            // No parity.
                      Sysvar[0x01220300 + 11] = 115200;       // Baudrate.


                      SmPeriod(100,id, TickPeriod);
                      return(SmTrans(->Sending));

               }
          SmState Sending {
                            SIG_ENTRY = {
                                           print (" Go Sending ");
                                         }
                          TickPeriod = {

                                          msg_S[0] = 0xEE;
                                          msg_S[1] = 0xFF;
                                          msg_S[2] = 0xFC;
                                          msg_S[3] = 0xFF;
                                          msg_S[4] = 0xFF;
                                          SerialWriteComm(1,msg_S);
                                         print("Message Sent: ",msg_S[0]," ",msg_S[1]," ",msg_S[2]," ",msg_S[3]," ",msg_S[4]);

                                         }

                          }

    }

SmMachine SmMsgRecvd { 2,*,Root, 20, 1 }

SmMachine SmMsgSent { 3, *, Root1,20, 2 }







long main(void)
{
    long res;

    res = SmRun(SmMsgSent,SmMsgRecvd);
    return(0);


}

//$X {User Parameter (1),1,1,0,-1,0,-1,0,(-1),-1},0x2201,1,0
//$X {User Parameter (2),1,1,0,-1,0,-1,0,(-1),-1},0x2201,2,0
//$X {User Parameter (3),1,1,0,-1,0,-1,0,(-1),-1},0x2201,3,0
//$X {User Parameter (4),1,1,0,-1,0,-1,0,(-1),-1},0x2201,4,0
//$X {User Parameter (99),1,1,0,-1,0,-1,0,(-1),-1},0x2201,99,0
//$X {User Parameter (1),1,1,0,-1,0,-1,0,(-1),-1},0x2201,1,0

#!/bin/bash

cansend vcan0 000000c1##0020104030605080704000002c002000042160a00d7200000948034c924000000040000000200000001000000feaf0000a196cbc824000000390d35c824000000
cansend vcan0 000000c1##0b159cbc8240000008ee4cbc824000000cb6eccc824000000ebf8ccc8240000006b83cdc824000000c80dcec8240000000b98cec8240000005b22cfc824000000
cansend vcan0 000000c1##0a6accfc8240000000037d0c82400000043c1d0c824000000934bd1c824000000ded5d1c8240000003860d2c8240000007bead2c824000000cb74d3c824000000
cansend vcan0 000000c1##016ffd3c8240000007089d4c824000000b313d5c824000000039ed5c8240000002228d6c824000000a2b2d6c824000000e83cd7c82400000044c7d7c824000000
cansend vcan0 000000c1##08751d8c824000000d7dbd8c8240000002266d9c8240000007cf0d9c824000000bf7adac8240000000f05dbc8240000005a8fdbc824000000b419dcc824000000
cansend vcan0 000000c1##0f7a3dcc824000000472eddc82400000092b8ddc824000000ec42dec8240000002fcddec8240000004b57dfc824000000c4e1dfc8240000001f6ce0c824000000
cansend vcan0 000000c1##066f6e0c824000000c080e1c824000000030be2c8240000005395e2c8240000009e1fe3c824000000f8a9e3c8240000003b34e4c8240000008bbee4c824000000
cansend vcan0 000000c1##0d648e5c82400000030d3e5c824000000735de6c824000000c3e7e6c8240000000e72e7c82400000068fce7c8240000007486e8c8240000000211e9c824000000
cansend vcan0 000000c1##0479be9c8240000009725eac824000000e2afeac8240000003c3aebc8240000007fc4ebc824000000cf4eecc8240000001ad9ecc824000000cccccccccccccccc

# Send frame header, type C1
# cansend vcan0 000000C1##00201040306050807040000022002000042160A0001000000F1AA3E690E0000000300000001000000FEAF0000D2D5C068CB96C068E373C168D550C268D72DC368
# cansend vcan0 000000C1##0D50AC468D5E7C468CDC4C568D9A1C668CF7EC768CB5BC868C138C968CB15CA68C1F2CA68CDCFCB68C3ACCC68BF89CD68BD66CE68BD43CF68B520D068C1FDD068
# cansend vcan0 000000C1##091DAD168B5B7D268AD94D368B971D468AF4ED568AB2BD668A908D768A9E5D768A1C2D868AD9FD968A37CDA689D59DB689736DC689713DD6893F0DD6893CDDE68
# cansend vcan0 000000C1##08FAADF688F87E0688B64E1688B41E268871EE3685FFBE3688FD8E4688DB5E5687B92E668876FE7687D4CE8688929E9687F06EA687BE3EA6879C0EB68799DEC68
# cansend vcan0 000000C1##04F7AED686F57EE686B34EF686B11F06867EEF06867CBF16863A8F2686385F3685F62F4685F3FF5685B1CF66867F9F668


# Send TLV, type D1
cansend vcan0 000000D1##00100000074000000
cansend vcan0 000000D1##00E000700C5FA7D13F8FF2100000097136C00CA00000052136B00D0000000A9136900D8000000EF126600DF000000CB125B00EA0000006C125C00F00000001112
cansend vcan0 000000D1##06701840100009B126C0186010000DF114200180200003D125D01F7010000E9115E01FE0100008C0E540055030000DF117A018201CCCCCCCCCCCCCCCCCCCCCCCC

# Send TLV, type D7
cansend vcan0 000000D7##00200000014000000
cansend vcan0 000000D7##0020007006400DC00090013006801B30112004A00

# Send TLV, type D8
cansend vcan0 000000D8##00400000044000000
cansend vcan0 000000D8##020000700000A000A000A000A000A000A000A000A000A000A000A000A000A000A000A000A000A000A000A000A000A000A000A000A000A000A000A000A000A000A
cansend vcan0 000000D8##0000A000A
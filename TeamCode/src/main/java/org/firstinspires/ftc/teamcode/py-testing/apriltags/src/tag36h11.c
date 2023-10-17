/* (C) 2013-2015, The Regents of The University of Michigan
All rights reserved.

This software may be available under alternative licensing
terms. Contact Edwin Olson, ebolson@umich.edu, for more information.

   An unlimited license is granted to use, adapt, modify, or embed the 2D
barcodes into any medium.

   Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:

1. Redistributions of source code must retain the above copyright notice, this
   list of conditions and the following disclaimer.
2. Redistributions in binary form must reproduce the above copyright notice,
   this list of conditions and the following disclaimer in the documentation
   and/or other materials provided with the distribution.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR
ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
(INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
(INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

The views and conclusions contained in the software and documentation are those
of the authors and should not be interpreted as representing official policies,
either expressed or implied, of the FreeBSD Project.
 */

#include <stdlib.h>
#include "apriltag.h"

apriltag_family_t *tag36h11_create()
{
   apriltag_family_t *tf = calloc(1, sizeof(apriltag_family_t));
   tf->name = strdup("tag36h11");
   tf->black_border = 1;
   tf->d = 6;
   tf->h = 11;
   tf->ncodes = 587;
   tf->codes = calloc(587, sizeof(uint64_t));
   tf->codes[0] = 0x0000000d5d628584UL;
   tf->codes[1] = 0x0000000d97f18b49UL;
   tf->codes[2] = 0x0000000dd280910eUL;
   tf->codes[3] = 0x0000000e479e9c98UL;
   tf->codes[4] = 0x0000000ebcbca822UL;
   tf->codes[5] = 0x0000000f31dab3acUL;
   tf->codes[6] = 0x0000000056a5d085UL;
   tf->codes[7] = 0x000000010652e1d4UL;
   tf->codes[8] = 0x000000022b1dfeadUL;
   tf->codes[9] = 0x0000000265ad0472UL;
   tf->codes[10] = 0x000000034fe91b86UL;
   tf->codes[11] = 0x00000003ff962cd5UL;
   tf->codes[12] = 0x000000043a25329aUL;
   tf->codes[13] = 0x0000000474b4385fUL;
   tf->codes[14] = 0x00000004e9d243e9UL;
   tf->codes[15] = 0x00000005246149aeUL;
   tf->codes[16] = 0x00000005997f5538UL;
   tf->codes[17] = 0x0000000683bb6c4cUL;
   tf->codes[18] = 0x00000006be4a7211UL;
   tf->codes[19] = 0x00000007e3158eeaUL;
   tf->codes[20] = 0x000000081da494afUL;
   tf->codes[21] = 0x0000000858339a74UL;
   tf->codes[22] = 0x00000008cd51a5feUL;
   tf->codes[23] = 0x00000009f21cc2d7UL;
   tf->codes[24] = 0x0000000a2cabc89cUL;
   tf->codes[25] = 0x0000000adc58d9ebUL;
   tf->codes[26] = 0x0000000b16e7dfb0UL;
   tf->codes[27] = 0x0000000b8c05eb3aUL;
   tf->codes[28] = 0x0000000d25ef139dUL;
   tf->codes[29] = 0x0000000d607e1962UL;
   tf->codes[30] = 0x0000000e4aba3076UL;
   tf->codes[31] = 0x00000002dde6a3daUL;
   tf->codes[32] = 0x000000043d40c678UL;
   tf->codes[33] = 0x00000005620be351UL;
   tf->codes[34] = 0x000000064c47fa65UL;
   tf->codes[35] = 0x0000000686d7002aUL;
   tf->codes[36] = 0x00000006c16605efUL;
   tf->codes[37] = 0x00000006fbf50bb4UL;
   tf->codes[38] = 0x00000008d06d39dcUL;
   tf->codes[39] = 0x00000009f53856b5UL;
   tf->codes[40] = 0x0000000adf746dc9UL;
   tf->codes[41] = 0x0000000bc9b084ddUL;
   tf->codes[42] = 0x0000000d290aa77bUL;
   tf->codes[43] = 0x0000000d9e28b305UL;
   tf->codes[44] = 0x0000000e4dd5c454UL;
   tf->codes[45] = 0x0000000fad2fe6f2UL;
   tf->codes[46] = 0x0000000181a8151aUL;
   tf->codes[47] = 0x000000026be42c2eUL;
   tf->codes[48] = 0x00000002e10237b8UL;
   tf->codes[49] = 0x0000000405cd5491UL;
   tf->codes[50] = 0x00000007742eab1cUL;
   tf->codes[51] = 0x000000085e6ac230UL;
   tf->codes[52] = 0x00000008d388cdbaUL;
   tf->codes[53] = 0x00000009f853ea93UL;
   tf->codes[54] = 0x0000000c41ea2445UL;
   tf->codes[55] = 0x0000000cf1973594UL;
   tf->codes[56] = 0x000000014a34a333UL;
   tf->codes[57] = 0x000000031eacd15bUL;
   tf->codes[58] = 0x00000006c79d2dabUL;
   tf->codes[59] = 0x000000073cbb3935UL;
   tf->codes[60] = 0x000000089c155bd3UL;
   tf->codes[61] = 0x00000008d6a46198UL;
   tf->codes[62] = 0x000000091133675dUL;
   tf->codes[63] = 0x0000000a708d89fbUL;
   tf->codes[64] = 0x0000000ae5ab9585UL;
   tf->codes[65] = 0x0000000b9558a6d4UL;
   tf->codes[66] = 0x0000000b98743ab2UL;
   tf->codes[67] = 0x0000000d6cec68daUL;
   tf->codes[68] = 0x00000001506bcaefUL;
   tf->codes[69] = 0x00000004becd217aUL;
   tf->codes[70] = 0x00000004f95c273fUL;
   tf->codes[71] = 0x0000000658b649ddUL;
   tf->codes[72] = 0x0000000a76c4b1b7UL;
   tf->codes[73] = 0x0000000ecf621f56UL;
   tf->codes[74] = 0x00000001c8a56a57UL;
   tf->codes[75] = 0x00000003628e92baUL;
   tf->codes[76] = 0x000000053706c0e2UL;
   tf->codes[77] = 0x00000005e6b3d231UL;
   tf->codes[78] = 0x00000007809cfa94UL;
   tf->codes[79] = 0x0000000e97eead6fUL;
   tf->codes[80] = 0x00000005af40604aUL;
   tf->codes[81] = 0x00000007492988adUL;
   tf->codes[82] = 0x0000000ed5994712UL;
   tf->codes[83] = 0x00000005eceaf9edUL;
   tf->codes[84] = 0x00000007c1632815UL;
   tf->codes[85] = 0x0000000c1a0095b4UL;
   tf->codes[86] = 0x0000000e9e25d52bUL;
   tf->codes[87] = 0x00000003a6705419UL;
   tf->codes[88] = 0x0000000a8333012fUL;
   tf->codes[89] = 0x00000004ce5704d0UL;
   tf->codes[90] = 0x0000000508e60a95UL;
   tf->codes[91] = 0x0000000877476120UL;
   tf->codes[92] = 0x0000000a864e950dUL;
   tf->codes[93] = 0x0000000ea45cfce7UL;
   tf->codes[94] = 0x000000019da047e8UL;
   tf->codes[95] = 0x000000024d4d5937UL;
   tf->codes[96] = 0x00000006e079cc9bUL;
   tf->codes[97] = 0x000000099f2e11d7UL;
   tf->codes[98] = 0x000000033aa50429UL;
   tf->codes[99] = 0x0000000499ff26c7UL;
   tf->codes[100] = 0x000000050f1d3251UL;
   tf->codes[101] = 0x000000066e7754efUL;
   tf->codes[102] = 0x000000096ad633ceUL;
   tf->codes[103] = 0x00000009a5653993UL;
   tf->codes[104] = 0x0000000aca30566cUL;
   tf->codes[105] = 0x0000000c298a790aUL;
   tf->codes[106] = 0x00000008be44b65dUL;
   tf->codes[107] = 0x0000000dc68f354bUL;
   tf->codes[108] = 0x000000016f7f919bUL;
   tf->codes[109] = 0x00000004dde0e826UL;
   tf->codes[110] = 0x0000000d548cbd9fUL;
   tf->codes[111] = 0x0000000e0439ceeeUL;
   tf->codes[112] = 0x0000000fd8b1fd16UL;
   tf->codes[113] = 0x000000076521bb7bUL;
   tf->codes[114] = 0x0000000d92375742UL;
   tf->codes[115] = 0x0000000cab16d40cUL;
   tf->codes[116] = 0x0000000730c9dd72UL;
   tf->codes[117] = 0x0000000ad9ba39c2UL;
   tf->codes[118] = 0x0000000b14493f87UL;
   tf->codes[119] = 0x000000052b15651fUL;
   tf->codes[120] = 0x0000000185409cadUL;
   tf->codes[121] = 0x000000077ae2c68dUL;
   tf->codes[122] = 0x000000094f5af4b5UL;
   tf->codes[123] = 0x00000000a13bad55UL;
   tf->codes[124] = 0x000000061ea437cdUL;
   tf->codes[125] = 0x0000000a022399e2UL;
   tf->codes[126] = 0x0000000203b163d1UL;
   tf->codes[127] = 0x00000007bba8f40eUL;
   tf->codes[128] = 0x000000095bc9442dUL;
   tf->codes[129] = 0x000000041c0b5358UL;
   tf->codes[130] = 0x00000008e9c6cc81UL;
   tf->codes[131] = 0x00000000eb549670UL;
   tf->codes[132] = 0x00000009da3a0b51UL;
   tf->codes[133] = 0x0000000d832a67a1UL;
   tf->codes[134] = 0x0000000dcd4350bcUL;
   tf->codes[135] = 0x00000004aa05fdd2UL;
   tf->codes[136] = 0x000000060c7bb44eUL;
   tf->codes[137] = 0x00000004b358b96cUL;
   tf->codes[138] = 0x0000000067299b45UL;
   tf->codes[139] = 0x0000000b9c89b5faUL;
   tf->codes[140] = 0x00000006975acaeaUL;
   tf->codes[141] = 0x000000062b8f7afaUL;
   tf->codes[142] = 0x000000033567c3d7UL;
   tf->codes[143] = 0x0000000bac139950UL;
   tf->codes[144] = 0x0000000a5927c62aUL;
   tf->codes[145] = 0x00000005c916e6a4UL;
   tf->codes[146] = 0x0000000260ecb7d5UL;
   tf->codes[147] = 0x000000029b7bbd9aUL;
   tf->codes[148] = 0x0000000903205f26UL;
   tf->codes[149] = 0x0000000ae72270a4UL;
   tf->codes[150] = 0x00000003d2ec51a7UL;
   tf->codes[151] = 0x000000082ea55324UL;
   tf->codes[152] = 0x000000011a6f3427UL;
   tf->codes[153] = 0x00000001ca1c4576UL;
   tf->codes[154] = 0x0000000a40c81aefUL;
   tf->codes[155] = 0x0000000bddccd730UL;
   tf->codes[156] = 0x00000000e617561eUL;
   tf->codes[157] = 0x0000000969317b0fUL;
   tf->codes[158] = 0x000000067f781364UL;
   tf->codes[159] = 0x0000000610912f96UL;
   tf->codes[160] = 0x0000000b2549fdfcUL;
   tf->codes[161] = 0x000000006e5aaa6bUL;
   tf->codes[162] = 0x0000000b6c475339UL;
   tf->codes[163] = 0x0000000c56836a4dUL;
   tf->codes[164] = 0x0000000844e351ebUL;
   tf->codes[165] = 0x00000004647f83b4UL;
   tf->codes[166] = 0x00000000908a04f5UL;
   tf->codes[167] = 0x00000007f51034c9UL;
   tf->codes[168] = 0x0000000aee537fcaUL;
   tf->codes[169] = 0x00000005e92494baUL;
   tf->codes[170] = 0x0000000d445808f4UL;
   tf->codes[171] = 0x000000028d68b563UL;
   tf->codes[172] = 0x000000004d25374bUL;
   tf->codes[173] = 0x00000002bc065f65UL;
   tf->codes[174] = 0x000000096dc3ea0cUL;
   tf->codes[175] = 0x00000004b2ade817UL;
   tf->codes[176] = 0x000000007c3fd502UL;
   tf->codes[177] = 0x0000000e768b5cafUL;
   tf->codes[178] = 0x000000017605cf6cUL;
   tf->codes[179] = 0x0000000182741ee4UL;
   tf->codes[180] = 0x000000062846097cUL;
   tf->codes[181] = 0x000000072b5ebf80UL;
   tf->codes[182] = 0x0000000263da6e13UL;
   tf->codes[183] = 0x0000000fa841bcb5UL;
   tf->codes[184] = 0x00000007e45e8c69UL;
   tf->codes[185] = 0x0000000653c81fa0UL;
   tf->codes[186] = 0x00000007443b5e70UL;
   tf->codes[187] = 0x00000000a5234afdUL;
   tf->codes[188] = 0x000000074756f24eUL;
   tf->codes[189] = 0x0000000157ebf02aUL;
   tf->codes[190] = 0x000000082ef46939UL;
   tf->codes[191] = 0x000000080d420264UL;
   tf->codes[192] = 0x00000002aeed3e98UL;
   tf->codes[193] = 0x0000000b0a1dd4f8UL;
   tf->codes[194] = 0x0000000b5436be13UL;
   tf->codes[195] = 0x00000007b7b4b13bUL;
   tf->codes[196] = 0x00000001ce80d6d3UL;
   tf->codes[197] = 0x000000016c08427dUL;
   tf->codes[198] = 0x0000000ee54462ddUL;
   tf->codes[199] = 0x00000001f7644cceUL;
   tf->codes[200] = 0x00000009c7b5cc92UL;
   tf->codes[201] = 0x0000000e369138f8UL;
   tf->codes[202] = 0x00000005d5a66e91UL;
   tf->codes[203] = 0x0000000485d62f49UL;
   tf->codes[204] = 0x0000000e6e819e94UL;
   tf->codes[205] = 0x0000000b1f340eb5UL;
   tf->codes[206] = 0x000000009d198ce2UL;
   tf->codes[207] = 0x0000000d60717437UL;
   tf->codes[208] = 0x00000000196b856cUL;
   tf->codes[209] = 0x0000000f0a6173a5UL;
   tf->codes[210] = 0x000000012c0e1ec6UL;
   tf->codes[211] = 0x000000062b82d5cfUL;
   tf->codes[212] = 0x0000000ad154c067UL;
   tf->codes[213] = 0x0000000ce3778832UL;
   tf->codes[214] = 0x00000006b0a7b864UL;
   tf->codes[215] = 0x00000004c7686694UL;
   tf->codes[216] = 0x00000005058ff3ecUL;
   tf->codes[217] = 0x0000000d5e21ea23UL;
   tf->codes[218] = 0x00000009ff4a76eeUL;
   tf->codes[219] = 0x00000009dd981019UL;
   tf->codes[220] = 0x00000001bad4d30aUL;
   tf->codes[221] = 0x0000000c601896d1UL;
   tf->codes[222] = 0x0000000973439b48UL;
   tf->codes[223] = 0x00000001ce7431a8UL;
   tf->codes[224] = 0x000000057a8021d6UL;
   tf->codes[225] = 0x0000000f9dba96e6UL;
   tf->codes[226] = 0x000000083a2e4e7cUL;
   tf->codes[227] = 0x00000008ea585380UL;
   tf->codes[228] = 0x0000000af6c0e744UL;
   tf->codes[229] = 0x0000000875b73babUL;
   tf->codes[230] = 0x0000000da34ca901UL;
   tf->codes[231] = 0x00000002ab9727efUL;
   tf->codes[232] = 0x0000000d39f21b9aUL;
   tf->codes[233] = 0x00000008a10b742fUL;
   tf->codes[234] = 0x00000005f8952dbaUL;
   tf->codes[235] = 0x0000000f8da71ab0UL;
   tf->codes[236] = 0x0000000c25f9df96UL;
   tf->codes[237] = 0x000000006f8a5d94UL;
   tf->codes[238] = 0x0000000e42e63e1aUL;
   tf->codes[239] = 0x0000000b78409d1bUL;
   tf->codes[240] = 0x0000000792229addUL;
   tf->codes[241] = 0x00000005acf8c455UL;
   tf->codes[242] = 0x00000002fc29a9b0UL;
   tf->codes[243] = 0x0000000ea486237bUL;
   tf->codes[244] = 0x0000000b0c9685a0UL;
   tf->codes[245] = 0x00000001ad748a47UL;
   tf->codes[246] = 0x000000003b4712d5UL;
   tf->codes[247] = 0x0000000f29216d30UL;
   tf->codes[248] = 0x00000008dad65e49UL;
   tf->codes[249] = 0x00000000a2cf09ddUL;
   tf->codes[250] = 0x00000000b5f174c6UL;
   tf->codes[251] = 0x0000000e54f57743UL;
   tf->codes[252] = 0x0000000b9cf54d78UL;
   tf->codes[253] = 0x00000004a312a88aUL;
   tf->codes[254] = 0x000000027babc962UL;
   tf->codes[255] = 0x0000000b86897111UL;
   tf->codes[256] = 0x0000000f2ff6c116UL;
   tf->codes[257] = 0x000000082274bd8aUL;
   tf->codes[258] = 0x000000097023505eUL;
   tf->codes[259] = 0x000000052d46edd1UL;
   tf->codes[260] = 0x0000000585c1f538UL;
   tf->codes[261] = 0x0000000bddd00e43UL;
   tf->codes[262] = 0x00000005590b74dfUL;
   tf->codes[263] = 0x0000000729404a1fUL;
   tf->codes[264] = 0x000000065320855eUL;
   tf->codes[265] = 0x0000000d3d4b6956UL;
   tf->codes[266] = 0x00000007ae374f14UL;
   tf->codes[267] = 0x00000002d7a60e06UL;
   tf->codes[268] = 0x0000000315cd9b5eUL;
   tf->codes[269] = 0x0000000fd36b4eacUL;
   tf->codes[270] = 0x0000000f1df7642bUL;
   tf->codes[271] = 0x000000055db27726UL;
   tf->codes[272] = 0x00000008f15ebc19UL;
   tf->codes[273] = 0x0000000992f8c531UL;
   tf->codes[274] = 0x000000062dea2a40UL;
   tf->codes[275] = 0x0000000928275cabUL;
   tf->codes[276] = 0x000000069c263cb9UL;
   tf->codes[277] = 0x0000000a774cca9eUL;
   tf->codes[278] = 0x0000000266b2110eUL;
   tf->codes[279] = 0x00000001b14acbb8UL;
   tf->codes[280] = 0x0000000624b8a71bUL;
   tf->codes[281] = 0x00000001c539406bUL;
   tf->codes[282] = 0x00000003086d529bUL;
   tf->codes[283] = 0x00000000111dd66eUL;
   tf->codes[284] = 0x000000098cd630bfUL;
   tf->codes[285] = 0x00000008b9d1ffdcUL;
   tf->codes[286] = 0x000000072b2f61e7UL;
   tf->codes[287] = 0x00000009ed9d672bUL;
   tf->codes[288] = 0x000000096cdd15f3UL;
   tf->codes[289] = 0x00000006366c2504UL;
   tf->codes[290] = 0x00000006ca9df73aUL;
   tf->codes[291] = 0x0000000a066d60f0UL;
   tf->codes[292] = 0x0000000e7a4b8addUL;
   tf->codes[293] = 0x00000008264647efUL;
   tf->codes[294] = 0x0000000aa195bf81UL;
   tf->codes[295] = 0x00000009a3db8244UL;
   tf->codes[296] = 0x0000000014d2df6aUL;
   tf->codes[297] = 0x00000000b63265b7UL;
   tf->codes[298] = 0x00000002f010de73UL;
   tf->codes[299] = 0x000000097e774986UL;
   tf->codes[300] = 0x0000000248affc29UL;
   tf->codes[301] = 0x0000000fb57dcd11UL;
   tf->codes[302] = 0x00000000b1a7e4d9UL;
   tf->codes[303] = 0x00000004bfa2d07dUL;
   tf->codes[304] = 0x000000054e5cdf96UL;
   tf->codes[305] = 0x00000004c15c1c86UL;
   tf->codes[306] = 0x0000000cd9c61166UL;
   tf->codes[307] = 0x0000000499380b2aUL;
   tf->codes[308] = 0x0000000540308d09UL;
   tf->codes[309] = 0x00000008b63fe66fUL;
   tf->codes[310] = 0x0000000c81aeb35eUL;
   tf->codes[311] = 0x000000086fe0bd5cUL;
   tf->codes[312] = 0x0000000ce2480c2aUL;
   tf->codes[313] = 0x00000001ab29ee60UL;
   tf->codes[314] = 0x00000008048daa15UL;
   tf->codes[315] = 0x0000000dbfeb2d39UL;
   tf->codes[316] = 0x0000000567c9858cUL;
   tf->codes[317] = 0x00000002b6edc5bcUL;
   tf->codes[318] = 0x00000002078fca82UL;
   tf->codes[319] = 0x0000000adacc22aaUL;
   tf->codes[320] = 0x0000000b92486f49UL;
   tf->codes[321] = 0x000000051fac5964UL;
   tf->codes[322] = 0x0000000691ee6420UL;
   tf->codes[323] = 0x0000000f63b3e129UL;
   tf->codes[324] = 0x000000039be7e572UL;
   tf->codes[325] = 0x0000000da2ce6c74UL;
   tf->codes[326] = 0x000000020cf17a5cUL;
   tf->codes[327] = 0x0000000ee55f9b6eUL;
   tf->codes[328] = 0x0000000fb8572726UL;
   tf->codes[329] = 0x0000000b2c2de548UL;
   tf->codes[330] = 0x0000000caa9bce92UL;
   tf->codes[331] = 0x0000000ae9182db3UL;
   tf->codes[332] = 0x000000074b6e5bd1UL;
   tf->codes[333] = 0x0000000137b252afUL;
   tf->codes[334] = 0x000000051f686881UL;
   tf->codes[335] = 0x0000000d672f6c02UL;
   tf->codes[336] = 0x0000000654146ce4UL;
   tf->codes[337] = 0x0000000f944bc825UL;
   tf->codes[338] = 0x0000000e8327f809UL;
   tf->codes[339] = 0x000000076a73fd59UL;
   tf->codes[340] = 0x0000000f79da4cb4UL;
   tf->codes[341] = 0x0000000956f8099bUL;
   tf->codes[342] = 0x00000007b5f2655cUL;
   tf->codes[343] = 0x0000000d06b114a6UL;
   tf->codes[344] = 0x0000000d0697ca50UL;
   tf->codes[345] = 0x000000027c390797UL;
   tf->codes[346] = 0x0000000bc61ed9b2UL;
   tf->codes[347] = 0x0000000cc12dd19bUL;
   tf->codes[348] = 0x0000000eb7818d2cUL;
   tf->codes[349] = 0x0000000092fcecdaUL;
   tf->codes[350] = 0x000000089ded4ea1UL;
   tf->codes[351] = 0x0000000256a0ba34UL;
   tf->codes[352] = 0x0000000b6948e627UL;
   tf->codes[353] = 0x00000001ef6b1054UL;
   tf->codes[354] = 0x00000008639294a2UL;
   tf->codes[355] = 0x0000000eda3780a4UL;
   tf->codes[356] = 0x000000039ee2af1dUL;
   tf->codes[357] = 0x0000000cd257edc5UL;
   tf->codes[358] = 0x00000002d9d6bc22UL;
   tf->codes[359] = 0x0000000121d3b47dUL;
   tf->codes[360] = 0x000000037e23f8adUL;
   tf->codes[361] = 0x0000000119f31cf6UL;
   tf->codes[362] = 0x00000002c97f4f09UL;
   tf->codes[363] = 0x0000000d502abfe0UL;
   tf->codes[364] = 0x000000010bc3ca77UL;
   tf->codes[365] = 0x000000053d7190efUL;
   tf->codes[366] = 0x000000090c3e62a6UL;
   tf->codes[367] = 0x00000007e9ebf675UL;
   tf->codes[368] = 0x0000000979ce23d1UL;
   tf->codes[369] = 0x000000027f0c98e9UL;
   tf->codes[370] = 0x0000000eafb4ae59UL;
   tf->codes[371] = 0x00000007ca7fe2bdUL;
   tf->codes[372] = 0x00000001490ca8f6UL;
   tf->codes[373] = 0x00000009123387baUL;
   tf->codes[374] = 0x0000000b3bc73888UL;
   tf->codes[375] = 0x00000003ea87e325UL;
   tf->codes[376] = 0x00000004888964aaUL;
   tf->codes[377] = 0x0000000a0188a6b9UL;
   tf->codes[378] = 0x0000000cd383c666UL;
   tf->codes[379] = 0x000000040029a3fdUL;
   tf->codes[380] = 0x0000000e1c00ac5cUL;
   tf->codes[381] = 0x000000039e6f2b6eUL;
   tf->codes[382] = 0x0000000de664f622UL;
   tf->codes[383] = 0x0000000e979a75e8UL;
   tf->codes[384] = 0x00000007c6b4c86cUL;
   tf->codes[385] = 0x0000000fd492e071UL;
   tf->codes[386] = 0x00000008fbb35118UL;
   tf->codes[387] = 0x000000040b4a09b7UL;
   tf->codes[388] = 0x0000000af80bd6daUL;
   tf->codes[389] = 0x000000070e0b2521UL;
   tf->codes[390] = 0x00000002f5c54d93UL;
   tf->codes[391] = 0x00000003f4a118d5UL;
   tf->codes[392] = 0x000000009c1897b9UL;
   tf->codes[393] = 0x0000000079776eacUL;
   tf->codes[394] = 0x0000000084b00b17UL;
   tf->codes[395] = 0x00000003a95ad90eUL;
   tf->codes[396] = 0x000000028c544095UL;
   tf->codes[397] = 0x000000039d457c05UL;
   tf->codes[398] = 0x00000007a3791a78UL;
   tf->codes[399] = 0x0000000bb770e22eUL;
   tf->codes[400] = 0x00000009a822bd6cUL;
   tf->codes[401] = 0x000000068a4b1fedUL;
   tf->codes[402] = 0x0000000a5fd27b3bUL;
   tf->codes[403] = 0x00000000c3995b79UL;
   tf->codes[404] = 0x0000000d1519dff1UL;
   tf->codes[405] = 0x00000008e7eee359UL;
   tf->codes[406] = 0x0000000cd3ca50b1UL;
   tf->codes[407] = 0x0000000b73b8b793UL;
   tf->codes[408] = 0x000000057aca1c43UL;
   tf->codes[409] = 0x0000000ec2655277UL;
   tf->codes[410] = 0x0000000785a2c1b3UL;
   tf->codes[411] = 0x000000075a07985aUL;
   tf->codes[412] = 0x0000000a4b01eb69UL;
   tf->codes[413] = 0x0000000a18a11347UL;
   tf->codes[414] = 0x0000000db1f28ca3UL;
   tf->codes[415] = 0x0000000877ec3e25UL;
   tf->codes[416] = 0x000000031f6341b8UL;
   tf->codes[417] = 0x00000001363a3a4cUL;
   tf->codes[418] = 0x0000000075d8b9baUL;
   tf->codes[419] = 0x00000007ae0792a9UL;
   tf->codes[420] = 0x0000000a83a21651UL;
   tf->codes[421] = 0x00000007f08f9fb5UL;
   tf->codes[422] = 0x00000000d0cf73a9UL;
   tf->codes[423] = 0x0000000b04dcc98eUL;
   tf->codes[424] = 0x0000000f65c7b0f8UL;
   tf->codes[425] = 0x000000065ddaf69aUL;
   tf->codes[426] = 0x00000002cf9b86b3UL;
   tf->codes[427] = 0x000000014cb51e25UL;
   tf->codes[428] = 0x0000000f48027b5bUL;
   tf->codes[429] = 0x00000000ec26ea8bUL;
   tf->codes[430] = 0x000000044bafd45cUL;
   tf->codes[431] = 0x0000000b12c7c0c4UL;
   tf->codes[432] = 0x0000000959fd9d82UL;
   tf->codes[433] = 0x0000000c77c9725aUL;
   tf->codes[434] = 0x000000048a22d462UL;
   tf->codes[435] = 0x00000008398e8072UL;
   tf->codes[436] = 0x0000000ec89b05ceUL;
   tf->codes[437] = 0x0000000bb682d4c9UL;
   tf->codes[438] = 0x0000000e5a86d2ffUL;
   tf->codes[439] = 0x0000000358f01134UL;
   tf->codes[440] = 0x00000008556ddcf6UL;
   tf->codes[441] = 0x000000067584b6e2UL;
   tf->codes[442] = 0x000000011609439fUL;
   tf->codes[443] = 0x000000008488816eUL;
   tf->codes[444] = 0x0000000aaf1a2c46UL;
   tf->codes[445] = 0x0000000f879898cfUL;
   tf->codes[446] = 0x00000008bbe5e2f7UL;
   tf->codes[447] = 0x0000000101eee363UL;
   tf->codes[448] = 0x0000000690f69377UL;
   tf->codes[449] = 0x0000000f5bd93cd9UL;
   tf->codes[450] = 0x0000000cea4c2bf6UL;
   tf->codes[451] = 0x00000009550be706UL;
   tf->codes[452] = 0x00000002c5b38a60UL;
   tf->codes[453] = 0x0000000e72033547UL;
   tf->codes[454] = 0x00000004458b0629UL;
   tf->codes[455] = 0x0000000ee8d9ed41UL;
   tf->codes[456] = 0x0000000d2f918d72UL;
   tf->codes[457] = 0x000000078dc39fd3UL;
   tf->codes[458] = 0x00000008212636f6UL;
   tf->codes[459] = 0x00000007450a72a7UL;
   tf->codes[460] = 0x0000000c4f0cf4c6UL;
   tf->codes[461] = 0x0000000367bcddcdUL;
   tf->codes[462] = 0x0000000c1caf8cc6UL;
   tf->codes[463] = 0x0000000a7f5b853dUL;
   tf->codes[464] = 0x00000009d536818bUL;
   tf->codes[465] = 0x0000000535e021b0UL;
   tf->codes[466] = 0x0000000a7eb8729eUL;
   tf->codes[467] = 0x0000000422a67b49UL;
   tf->codes[468] = 0x0000000929e928a6UL;
   tf->codes[469] = 0x000000048e8aefccUL;
   tf->codes[470] = 0x0000000a9897393cUL;
   tf->codes[471] = 0x00000005eb81d37eUL;
   tf->codes[472] = 0x00000001e80287b7UL;
   tf->codes[473] = 0x000000034770d903UL;
   tf->codes[474] = 0x00000002eef86728UL;
   tf->codes[475] = 0x000000059266ccb6UL;
   tf->codes[476] = 0x00000000110bba61UL;
   tf->codes[477] = 0x00000001dfd284efUL;
   tf->codes[478] = 0x0000000447439d1bUL;
   tf->codes[479] = 0x0000000fece0e599UL;
   tf->codes[480] = 0x00000009309f3703UL;
   tf->codes[481] = 0x000000080764d1ddUL;
   tf->codes[482] = 0x0000000353f1e6a0UL;
   tf->codes[483] = 0x00000002c1c12dccUL;
   tf->codes[484] = 0x0000000c1d21b9d7UL;
   tf->codes[485] = 0x0000000457ee453eUL;
   tf->codes[486] = 0x0000000d66faf540UL;
   tf->codes[487] = 0x000000044831e652UL;
   tf->codes[488] = 0x0000000cfd49a848UL;
   tf->codes[489] = 0x00000009312d4133UL;
   tf->codes[490] = 0x00000003f097d3eeUL;
   tf->codes[491] = 0x00000008c9ebef7aUL;
   tf->codes[492] = 0x0000000a99e29e88UL;
   tf->codes[493] = 0x00000000e9fab22cUL;
   tf->codes[494] = 0x00000004e748f4fbUL;
   tf->codes[495] = 0x0000000ecdee4288UL;
   tf->codes[496] = 0x0000000abce5f1d0UL;
   tf->codes[497] = 0x0000000c42f6876cUL;
   tf->codes[498] = 0x00000007ed402ea0UL;
   tf->codes[499] = 0x0000000e5c4242c3UL;
   tf->codes[500] = 0x0000000d5b2c31aeUL;
   tf->codes[501] = 0x0000000286863be6UL;
   tf->codes[502] = 0x0000000160444d94UL;
   tf->codes[503] = 0x00000005f0f5808eUL;
   tf->codes[504] = 0x0000000ae3d44b2aUL;
   tf->codes[505] = 0x00000009f5c5d109UL;
   tf->codes[506] = 0x00000008ad9316d7UL;
   tf->codes[507] = 0x00000003422ba064UL;
   tf->codes[508] = 0x00000002fed11d56UL;
   tf->codes[509] = 0x0000000bea6e3e04UL;
   tf->codes[510] = 0x000000004b029eecUL;
   tf->codes[511] = 0x00000006deed7435UL;
   tf->codes[512] = 0x00000003718ce17cUL;
   tf->codes[513] = 0x000000055857f5e2UL;
   tf->codes[514] = 0x00000002edac7b62UL;
   tf->codes[515] = 0x0000000085d6c512UL;
   tf->codes[516] = 0x0000000d6ca88e0fUL;
   tf->codes[517] = 0x00000002b7e1fc69UL;
   tf->codes[518] = 0x0000000a699d5c1bUL;
   tf->codes[519] = 0x0000000f05ad74deUL;
   tf->codes[520] = 0x00000004cf5fb56dUL;
   tf->codes[521] = 0x00000005725e07e1UL;
   tf->codes[522] = 0x000000072f18a2deUL;
   tf->codes[523] = 0x00000001cec52609UL;
   tf->codes[524] = 0x000000048534243cUL;
   tf->codes[525] = 0x00000002523a4d69UL;
   tf->codes[526] = 0x000000035c1b80d1UL;
   tf->codes[527] = 0x0000000a4d7338a7UL;
   tf->codes[528] = 0x00000000db1af012UL;
   tf->codes[529] = 0x0000000e61a9475dUL;
   tf->codes[530] = 0x000000005df03f91UL;
   tf->codes[531] = 0x000000097ae260bbUL;
   tf->codes[532] = 0x000000032d627fefUL;
   tf->codes[533] = 0x0000000b640f73c2UL;
   tf->codes[534] = 0x000000045a1ac9c6UL;
   tf->codes[535] = 0x00000006a2202de1UL;
   tf->codes[536] = 0x000000057d3e25f2UL;
   tf->codes[537] = 0x00000005aa9f986eUL;
   tf->codes[538] = 0x00000000cc859d8aUL;
   tf->codes[539] = 0x0000000e3ec6cca8UL;
   tf->codes[540] = 0x000000054e95e1aeUL;
   tf->codes[541] = 0x0000000446887b06UL;
   tf->codes[542] = 0x00000007516732beUL;
   tf->codes[543] = 0x00000003817ac8f5UL;
   tf->codes[544] = 0x00000003e26d938cUL;
   tf->codes[545] = 0x0000000aa81bc235UL;
   tf->codes[546] = 0x0000000df387ca1bUL;
   tf->codes[547] = 0x00000000f3a3b3f2UL;
   tf->codes[548] = 0x0000000b4bf69677UL;
   tf->codes[549] = 0x0000000ae21868edUL;
   tf->codes[550] = 0x000000081e1d2d9dUL;
   tf->codes[551] = 0x0000000a0a9ea14cUL;
   tf->codes[552] = 0x00000008eee297a9UL;
   tf->codes[553] = 0x00000004740c0559UL;
   tf->codes[554] = 0x0000000e8b141837UL;
   tf->codes[555] = 0x0000000ac69e0a3dUL;
   tf->codes[556] = 0x00000009ed83a1e1UL;
   tf->codes[557] = 0x00000005edb55ecbUL;
   tf->codes[558] = 0x000000007340fe81UL;
   tf->codes[559] = 0x000000050dfbc6bfUL;
   tf->codes[560] = 0x00000004f583508aUL;
   tf->codes[561] = 0x0000000cb1fb78bcUL;
   tf->codes[562] = 0x00000004025ced2fUL;
   tf->codes[563] = 0x000000039791ebecUL;
   tf->codes[564] = 0x000000053ee388f1UL;
   tf->codes[565] = 0x00000007d6c0bd23UL;
   tf->codes[566] = 0x000000093a995fbeUL;
   tf->codes[567] = 0x00000008a41728deUL;
   tf->codes[568] = 0x00000002fe70e053UL;
   tf->codes[569] = 0x0000000ab3db443aUL;
   tf->codes[570] = 0x00000001364edb05UL;
   tf->codes[571] = 0x000000047b6eeed6UL;
   tf->codes[572] = 0x000000012e71af01UL;
   tf->codes[573] = 0x000000052ff83587UL;
   tf->codes[574] = 0x00000003a1575dd8UL;
   tf->codes[575] = 0x00000003feaa3564UL;
   tf->codes[576] = 0x0000000eacf78ba7UL;
   tf->codes[577] = 0x00000000872b94f8UL;
   tf->codes[578] = 0x0000000da8ddf9a2UL;
   tf->codes[579] = 0x00000009aa920d2bUL;
   tf->codes[580] = 0x00000001f350ed36UL;
   tf->codes[581] = 0x000000018a5e861fUL;
   tf->codes[582] = 0x00000002c35b89c3UL;
   tf->codes[583] = 0x00000003347ac48aUL;
   tf->codes[584] = 0x00000007f23e022eUL;
   tf->codes[585] = 0x00000002459068fbUL;
   tf->codes[586] = 0x0000000e83be4b73UL;
   return tf;
}

void tag36h11_destroy(apriltag_family_t *tf)
{
   free(tf->name);
   free(tf->codes);
   free(tf);
}

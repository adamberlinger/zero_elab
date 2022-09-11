/* File generated by tools/gen_sin.m */
#if SAMPLE_TABLE_RESOLUTION == 16
static const uint16_t sin_samples[128] = {
32768,34376,35980,37576,39161,40730,42280,43807,45307,46778,
48214,49614,50972,52287,53555,54773,55938,57047,58097,59087,
60013,60873,61666,62389,63041,63620,64124,64553,64905,65180,
65377,65496,65535,65496,65377,65180,64905,64553,64124,63620,
63041,62389,61666,60873,60013,59087,58097,57047,55938,54773,
53555,52287,50972,49614,48214,46778,45307,43807,42280,40730,
39161,37576,35980,34376,32768,31160,29556,27960,26375,24806,
23256,21729,20229,18758,17322,15922,14564,13249,11981,10763,
9598,8489,7439,6449,5523,4663,3870,3147,2495,1916,
1412,983,631,356,159,40,1,40,159,356,
631,983,1412,1916,2495,3147,3870,4663,5523,6449,
7439,8489,9598,10763,11981,13249,14564,15922,17322,18758,
20229,21729,23256,24806,26375,27960,29556,31160,};
#elif SAMPLE_TABLE_RESOLUTION == 32
static const uint32_t sin_samples[128] = {
2147483648,2252855676,2357973854,2462584942,2566436924,2669279611,2770865245,2870949099,2969290061,3065651219,
3159800432,3251510884,3340561638,3426738163,3509832852,3589645522,3665983897,3738664073,3807510956,3872358687,
3933051043,3989441812,4041395141,4088785872,4131499836,4169434131,4202497370,4230609901,4253703999,4271724027,
4284626574,4292380557,4294967295,4292380557,4284626574,4271724027,4253703999,4230609901,4202497370,4169434131,
4131499836,4088785872,4041395141,3989441812,3933051043,3872358687,3807510956,3738664073,3665983897,3589645522,
3509832852,3426738163,3340561638,3251510884,3159800432,3065651219,2969290061,2870949099,2770865245,2669279611,
2566436924,2462584942,2357973854,2252855676,2147483648,2042111620,1936993442,1832382354,1728530372,1625687685,
1524102051,1424018197,1325677235,1229316077,1135166864,1043456412,954405658,868229133,785134444,705321774,
628983399,556303223,487456340,422608609,361916253,305525484,253572155,206181424,163467460,125533165,
92469926,64357395,41263297,23243269,10340722,2586739,1,2586739,10340722,23243269,
41263297,64357395,92469926,125533165,163467460,206181424,253572155,305525484,361916253,422608609,
487456340,556303223,628983399,705321774,785134444,868229133,954405658,1043456412,1135166864,1229316077,
1325677235,1424018197,1524102051,1625687685,1728530372,1832382354,1936993442,2042111620,};
#endif
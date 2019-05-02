#define V_LENGTH 9
#define P_LENGTH 5
#define PHI_IDX_LENGTH 12
#define Q_IDX_LENGTH 13

static const float V[V_LENGTH] = {-20.0,-1.5,-1.0,-0.5,0,0.5,1.0,1.5,2.0};

static const float P[P_LENGTH] = {0,90,180,270,360};

static const int8_t PHI_IDX[PHI_IDX_LENGTH] = {0,6,10,11,12,15,16,18,20,21,22,24};

static const float PHI[V_LENGTH][P_LENGTH][PHI_IDX_LENGTH] = {1.000000,1.000000,0.000000,-0.020000,1.000000,0.010000,0.000000,1.000000,0.000000,-0.000100,0.010000,1.000000,
1.000000,1.000000,0.000548,-0.019992,1.000000,0.009996,0.000274,1.000000,0.000000,-0.000100,0.010000,1.000000,
1.000000,1.000000,0.001096,-0.019970,1.000000,0.009985,0.000548,1.000000,0.000000,-0.000100,0.010000,1.000000,
1.000000,1.000000,0.001643,-0.019932,1.000000,0.009966,0.000822,1.000000,0.000000,-0.000100,0.010000,1.000000,
1.000000,1.000000,0.002189,-0.019880,1.000000,0.009940,0.001094,1.000000,0.000011,-0.000099,0.010000,1.000000,
1.000000,1.000000,0.000000,-0.015000,1.000000,0.010000,0.000000,1.000000,0.000000,-0.000075,0.010000,1.000000,
1.000000,1.000000,0.000411,-0.014994,1.000000,0.009996,0.000274,1.000000,0.000000,-0.000075,0.010000,1.000000,
1.000000,1.000000,0.000822,-0.014977,1.000000,0.009985,0.000548,1.000000,0.000000,-0.000075,0.010000,1.000000,
1.000000,1.000000,0.001232,-0.014949,1.000000,0.009966,0.000822,1.000000,0.000000,-0.000075,0.010000,1.000000,
1.000000,1.000000,0.001642,-0.014910,1.000000,0.009940,0.001094,1.000000,0.000000,-0.000075,0.010000,1.000000,
1.000000,1.000000,0.000000,-0.010000,1.000000,0.010000,0.000000,1.000000,0.000000,-0.000050,0.010000,1.000000,
1.000000,1.000000,0.000274,-0.009996,1.000000,0.009996,0.000274,1.000000,0.000000,-0.000050,0.010000,1.000000,
1.000000,1.000000,0.000548,-0.009985,1.000000,0.009985,0.000548,1.000000,0.000000,-0.000050,0.010000,1.000000,
1.000000,1.000000,0.000822,-0.009966,1.000000,0.009966,0.000822,1.000000,0.000000,-0.000050,0.010000,1.000000,
1.000000,1.000000,0.001094,-0.009940,1.000000,0.009940,0.001094,1.000000,0.000000,-0.000050,0.010000,1.000000,
1.000000,1.000000,0.000000,-0.005000,1.000000,0.010000,0.000000,1.000000,0.000000,-0.000025,0.010000,1.000000,
1.000000,1.000000,0.000137,-0.004998,1.000000,0.009996,0.000274,1.000000,0.000000,-0.000025,0.010000,1.000000,
1.000000,1.000000,0.000274,-0.004992,1.000000,0.009985,0.000548,1.000000,0.000000,-0.000025,0.010000,1.000000,
1.000000,1.000000,0.000411,-0.004983,1.000000,0.009966,0.000822,1.000000,0.000000,-0.000025,0.010000,1.000000,
1.000000,1.000000,0.000547,-0.004970,1.000000,0.009940,0.001094,1.000000,0.000000,-0.000025,0.010000,1.000000,
1.000000,1.000000,0.000000,0.000000,1.000000,0.010000,0.000000,1.000000,0.000000,0.000000,0.010000,1.000000,
1.000000,1.000000,0.000000,0.000000,1.000000,0.009996,0.000274,1.000000,0.000000,0.000000,0.010000,1.000000,
1.000000,1.000000,0.000000,0.000000,1.000000,0.009985,0.000548,1.000000,0.000000,0.000000,0.010000,1.000000,
1.000000,1.000000,0.000000,0.000000,1.000000,0.009966,0.000822,1.000000,0.000000,0.000000,0.010000,1.000000,
1.000000,1.000000,0.000000,0.000000,1.000000,0.009940,0.001094,1.000000,0.000000,0.000000,0.010000,1.000000,
1.000000,1.000000,0.000000,0.005000,1.000000,0.010000,0.000000,1.000000,0.000000,0.000025,0.010000,1.000000,
1.000000,1.000000,-0.000137,0.004998,1.000000,0.009996,0.000274,1.000000,0.000000,0.000025,0.010000,1.000000,
1.000000,1.000000,-0.000274,0.004992,1.000000,0.009985,0.000548,1.000000,0.000000,0.000025,0.010000,1.000000,
1.000000,1.000000,-0.000411,0.004983,1.000000,0.009966,0.000822,1.000000,0.000000,0.000025,0.010000,1.000000,
1.000000,1.000000,-0.000547,0.004970,1.000000,0.009940,0.001094,1.000000,0.000000,0.000025,0.010000,1.000000,
1.000000,1.000000,0.000000,0.010000,1.000000,0.010000,0.000000,1.000000,0.000000,0.000050,0.010000,1.000000,
1.000000,1.000000,-0.000274,0.009996,1.000000,0.009996,0.000274,1.000000,0.000000,0.000050,0.010000,1.000000,
1.000000,1.000000,-0.000548,0.009985,1.000000,0.009985,0.000548,1.000000,0.000000,0.000050,0.010000,1.000000,
1.000000,1.000000,-0.000822,0.009966,1.000000,0.009966,0.000822,1.000000,0.000000,0.000050,0.010000,1.000000,
1.000000,1.000000,-0.001094,0.009940,1.000000,0.009940,0.001094,1.000000,0.000000,0.000050,0.010000,1.000000,
1.000000,1.000000,0.000000,0.015000,1.000000,0.010000,0.000000,1.000000,0.000000,0.000075,0.010000,1.000000,
1.000000,1.000000,-0.000411,0.014994,1.000000,0.009996,0.000274,1.000000,0.000000,0.000075,0.010000,1.000000,
1.000000,1.000000,-0.000822,0.014977,1.000000,0.009985,0.000548,1.000000,0.000000,0.000075,0.010000,1.000000,
1.000000,1.000000,-0.001232,0.014949,1.000000,0.009966,0.000822,1.000000,0.000000,0.000075,0.010000,1.000000,
1.000000,1.000000,-0.001642,0.014910,1.000000,0.009940,0.001094,1.000000,0.000000,0.000075,0.010000,1.000000,
1.000000,1.000000,0.000000,0.020000,1.000000,0.010000,0.000000,1.000000,0.000000,0.000100,0.010000,1.000000,
1.000000,1.000000,-0.000548,0.019992,1.000000,0.009996,0.000274,1.000000,0.000000,0.000100,0.010000,1.000000,
1.000000,1.000000,-0.001096,0.019970,1.000000,0.009985,0.000548,1.000000,0.000000,0.000100,0.010000,1.000000,
1.000000,1.000000,-0.001643,0.019932,1.000000,0.009966,0.000822,1.000000,0.000000,0.000100,0.010000,1.000000,
1.000000,1.000000,-0.002189,0.019880,1.000000,0.009940,0.001094,1.000000,-0.000011,0.000099,0.010000,1.000000,
};

static const int8_t Q_IDX[Q_IDX_LENGTH] = {0,2,3,6,7,10,11,12,14,15,18,22,24};

/*static const float Q[V_LENGTH][P_LENGTH][Q_IDX_LENGTH] = {0.001000,0.000000,0.000010,0.001007,-0.000500,0.000000,-0.000500,0.050003,0.000500,0.000010,0.002000,0.000500,0.100000,
0.001000,0.000000,0.000000,0.001007,-0.000500,0.000000,-0.000500,0.050003,0.000500,0.000000,0.002000,0.000500,0.100000,
0.001000,0.000014,0.000000,0.001007,-0.000500,0.000014,-0.000500,0.050003,0.000500,0.000000,0.002000,0.000500,0.100000,
0.001000,0.000021,0.000000,0.001007,-0.000500,0.000021,-0.000500,0.050003,0.000500,0.000000,0.002000,0.000500,0.100000,
0.001000,0.000027,0.000000,0.001007,-0.000499,0.000027,-0.000499,0.050003,0.000500,0.000000,0.002000,0.000500,0.100000,
0.001000,0.000034,0.000000,0.001007,-0.000499,0.000034,-0.000499,0.050003,0.000500,0.000000,0.002000,0.000500,0.100000,
0.001000,0.000041,0.000000,0.001007,-0.000498,0.000041,-0.000498,0.050003,0.000500,0.000000,0.002000,0.000500,0.100000,
0.001000,0.000048,0.000000,0.001007,-0.000498,0.000048,-0.000498,0.050003,0.000500,0.000000,0.002000,0.000500,0.100000,
0.001000,0.000055,0.000000,0.001007,-0.000497,0.000055,-0.000497,0.050003,0.000500,0.000000,0.002000,0.000500,0.100000,
0.001000,0.000000,0.000010,0.001005,-0.000450,0.000000,-0.000450,0.050003,0.000500,0.000010,0.002000,0.000500,0.100000,
0.001000,0.000000,0.000000,0.001005,-0.000450,0.000000,-0.000450,0.050003,0.000500,0.000000,0.002000,0.000500,0.100000,
0.001000,0.000012,0.000000,0.001005,-0.000450,0.000012,-0.000450,0.050003,0.000500,0.000000,0.002000,0.000500,0.100000,
0.001000,0.000019,0.000000,0.001005,-0.000450,0.000019,-0.000450,0.050003,0.000500,0.000000,0.002000,0.000500,0.100000,
0.001000,0.000025,0.000000,0.001005,-0.000449,0.000025,-0.000449,0.050003,0.000500,0.000000,0.002000,0.000500,0.100000,
0.001000,0.000031,0.000000,0.001005,-0.000449,0.000031,-0.000449,0.050003,0.000500,0.000000,0.002000,0.000500,0.100000,
0.001000,0.000037,0.000000,0.001005,-0.000449,0.000037,-0.000449,0.050003,0.000500,0.000000,0.002000,0.000500,0.100000,
0.001000,0.000043,0.000000,0.001005,-0.000448,0.000043,-0.000448,0.050003,0.000500,0.000000,0.002000,0.000500,0.100000,
0.001000,0.000049,0.000000,0.001005,-0.000447,0.000049,-0.000447,0.050003,0.000500,0.000000,0.002000,0.000500,0.100000,
0.001000,0.000000,0.000010,0.001004,-0.000400,0.000000,-0.000400,0.050003,0.000500,0.000010,0.002000,0.000500,0.100000,
0.001000,0.000000,0.000000,0.001004,-0.000400,0.000000,-0.000400,0.050003,0.000500,0.000000,0.002000,0.000500,0.100000,
0.001000,0.000011,0.000000,0.001004,-0.000400,0.000011,-0.000400,0.050003,0.000500,0.000000,0.002000,0.000500,0.100000,
0.001000,0.000016,0.000000,0.001004,-0.000400,0.000016,-0.000400,0.050003,0.000500,0.000000,0.002000,0.000500,0.100000,
0.001000,0.000022,0.000000,0.001004,-0.000399,0.000022,-0.000399,0.050003,0.000500,0.000000,0.002000,0.000500,0.100000,
0.001000,0.000027,0.000000,0.001004,-0.000399,0.000027,-0.000399,0.050003,0.000500,0.000000,0.002000,0.000500,0.100000,
0.001000,0.000033,0.000000,0.001004,-0.000399,0.000033,-0.000399,0.050003,0.000500,0.000000,0.002000,0.000500,0.100000,
0.001000,0.000038,0.000000,0.001004,-0.000398,0.000038,-0.000398,0.050003,0.000500,0.000000,0.002000,0.000500,0.100000,
0.001000,0.000044,0.000000,0.001004,-0.000398,0.000044,-0.000398,0.050003,0.000500,0.000000,0.002000,0.000500,0.100000,
0.001000,0.000000,0.000010,0.001003,-0.000350,0.000000,-0.000350,0.050003,0.000500,0.000010,0.002000,0.000500,0.100000,
0.001000,0.000000,0.000000,0.001003,-0.000350,0.000000,-0.000350,0.050003,0.000500,0.000000,0.002000,0.000500,0.100000,
0.001000,0.000000,0.000000,0.001003,-0.000350,0.000000,-0.000350,0.050003,0.000500,0.000000,0.002000,0.000500,0.100000,
0.001000,0.000014,0.000000,0.001003,-0.000350,0.000014,-0.000350,0.050003,0.000500,0.000000,0.002000,0.000500,0.100000,
0.001000,0.000019,0.000000,0.001003,-0.000349,0.000019,-0.000349,0.050003,0.000500,0.000000,0.002000,0.000500,0.100000,
0.001000,0.000024,0.000000,0.001003,-0.000349,0.000024,-0.000349,0.050003,0.000500,0.000000,0.002000,0.000500,0.100000,
0.001000,0.000029,0.000000,0.001003,-0.000349,0.000029,-0.000349,0.050003,0.000500,0.000000,0.002000,0.000500,0.100000,
0.001000,0.000034,0.000000,0.001003,-0.000348,0.000034,-0.000348,0.050003,0.000500,0.000000,0.002000,0.000500,0.100000,
0.001000,0.000038,0.000000,0.001003,-0.000348,0.000038,-0.000348,0.050003,0.000500,0.000000,0.002000,0.000500,0.100000,
0.001000,0.000000,0.000010,0.001002,-0.000300,0.000000,-0.000300,0.050003,0.000500,0.000010,0.002000,0.000500,0.100000,
0.001000,0.000000,0.000000,0.001002,-0.000300,0.000000,-0.000300,0.050003,0.000500,0.000000,0.002000,0.000500,0.100000,
0.001000,0.000000,0.000000,0.001002,-0.000300,0.000000,-0.000300,0.050003,0.000500,0.000000,0.002000,0.000500,0.100000,
0.001000,0.000012,0.000000,0.001002,-0.000300,0.000012,-0.000300,0.050003,0.000500,0.000000,0.002000,0.000500,0.100000,
0.001000,0.000016,0.000000,0.001002,-0.000300,0.000016,-0.000300,0.050003,0.000500,0.000000,0.002000,0.000500,0.100000,
0.001000,0.000021,0.000000,0.001002,-0.000299,0.000021,-0.000299,0.050003,0.000500,0.000000,0.002000,0.000500,0.100000,
0.001000,0.000025,0.000000,0.001002,-0.000299,0.000025,-0.000299,0.050003,0.000500,0.000000,0.002000,0.000500,0.100000,
0.001000,0.000029,0.000000,0.001002,-0.000299,0.000029,-0.000299,0.050003,0.000500,0.000000,0.002000,0.000500,0.100000,
0.001000,0.000033,0.000000,0.001002,-0.000298,0.000033,-0.000298,0.050003,0.000500,0.000000,0.002000,0.000500,0.100000,
0.001000,0.000000,0.000010,0.001002,-0.000250,0.000000,-0.000250,0.050003,0.000500,0.000010,0.002000,0.000500,0.100000,
0.001000,0.000000,0.000000,0.001002,-0.000250,0.000000,-0.000250,0.050003,0.000500,0.000000,0.002000,0.000500,0.100000,
0.001000,0.000000,0.000000,0.001002,-0.000250,0.000000,-0.000250,0.050003,0.000500,0.000000,0.002000,0.000500,0.100000,
0.001000,0.000010,0.000000,0.001002,-0.000250,0.000010,-0.000250,0.050003,0.000500,0.000000,0.002000,0.000500,0.100000,
0.001000,0.000014,0.000000,0.001002,-0.000250,0.000014,-0.000250,0.050003,0.000500,0.000000,0.002000,0.000500,0.100000,
0.001000,0.000017,0.000000,0.001002,-0.000249,0.000017,-0.000249,0.050003,0.000500,0.000000,0.002000,0.000500,0.100000,
0.001000,0.000021,0.000000,0.001002,-0.000249,0.000021,-0.000249,0.050003,0.000500,0.000000,0.002000,0.000500,0.100000,
0.001000,0.000024,0.000000,0.001002,-0.000249,0.000024,-0.000249,0.050003,0.000500,0.000000,0.002000,0.000500,0.100000,
0.001000,0.000027,0.000000,0.001002,-0.000249,0.000027,-0.000249,0.050003,0.000500,0.000000,0.002000,0.000500,0.100000,
0.001000,0.000000,0.000010,0.001001,-0.000200,0.000000,-0.000200,0.050003,0.000500,0.000010,0.002000,0.000500,0.100000,
0.001000,0.000000,0.000000,0.001001,-0.000200,0.000000,-0.000200,0.050003,0.000500,0.000000,0.002000,0.000500,0.100000,
0.001000,0.000000,0.000000,0.001001,-0.000200,0.000000,-0.000200,0.050003,0.000500,0.000000,0.002000,0.000500,0.100000,
0.001000,0.000000,0.000000,0.001001,-0.000200,0.000000,-0.000200,0.050003,0.000500,0.000000,0.002000,0.000500,0.100000,
0.001000,0.000011,0.000000,0.001001,-0.000200,0.000011,-0.000200,0.050003,0.000500,0.000000,0.002000,0.000500,0.100000,
0.001000,0.000014,0.000000,0.001001,-0.000200,0.000014,-0.000200,0.050003,0.000500,0.000000,0.002000,0.000500,0.100000,
0.001000,0.000016,0.000000,0.001001,-0.000199,0.000016,-0.000199,0.050003,0.000500,0.000000,0.002000,0.000500,0.100000,
0.001000,0.000019,0.000000,0.001001,-0.000199,0.000019,-0.000199,0.050003,0.000500,0.000000,0.002000,0.000500,0.100000,
0.001000,0.000022,0.000000,0.001001,-0.000199,0.000022,-0.000199,0.050003,0.000500,0.000000,0.002000,0.000500,0.100000,
0.001000,0.000000,0.000010,0.001001,-0.000150,0.000000,-0.000150,0.050003,0.000500,0.000010,0.002000,0.000500,0.100000,
0.001000,0.000000,0.000000,0.001001,-0.000150,0.000000,-0.000150,0.050003,0.000500,0.000000,0.002000,0.000500,0.100000,
0.001000,0.000000,0.000000,0.001001,-0.000150,0.000000,-0.000150,0.050003,0.000500,0.000000,0.002000,0.000500,0.100000,
0.001000,0.000000,0.000000,0.001001,-0.000150,0.000000,-0.000150,0.050003,0.000500,0.000000,0.002000,0.000500,0.100000,
0.001000,0.000000,0.000000,0.001001,-0.000150,0.000000,-0.000150,0.050003,0.000500,0.000000,0.002000,0.000500,0.100000,
0.001000,0.000010,0.000000,0.001001,-0.000150,0.000010,-0.000150,0.050003,0.000500,0.000000,0.002000,0.000500,0.100000,
0.001000,0.000012,0.000000,0.001001,-0.000150,0.000012,-0.000150,0.050003,0.000500,0.000000,0.002000,0.000500,0.100000,
0.001000,0.000014,0.000000,0.001001,-0.000149,0.000014,-0.000149,0.050003,0.000500,0.000000,0.002000,0.000500,0.100000,
0.001000,0.000016,0.000000,0.001001,-0.000149,0.000016,-0.000149,0.050003,0.000500,0.000000,0.002000,0.000500,0.100000,
0.001000,0.000000,0.000000,0.001000,-0.000100,0.000000,-0.000100,0.050003,0.000500,0.000010,0.002000,0.000500,0.100000,
0.001000,0.000000,0.000000,0.001000,-0.000100,0.000000,-0.000100,0.050003,0.000500,0.000000,0.002000,0.000500,0.100000,
0.001000,0.000000,0.000000,0.001000,-0.000100,0.000000,-0.000100,0.050003,0.000500,0.000000,0.002000,0.000500,0.100000,
0.001000,0.000000,0.000000,0.001000,-0.000100,0.000000,-0.000100,0.050003,0.000500,0.000000,0.002000,0.000500,0.100000,
0.001000,0.000000,0.000000,0.001000,-0.000100,0.000000,-0.000100,0.050003,0.000500,0.000000,0.002000,0.000500,0.100000,
0.001000,0.000000,0.000000,0.001000,-0.000100,0.000000,-0.000100,0.050003,0.000500,0.000000,0.002000,0.000500,0.100000,
0.001000,0.000000,0.000000,0.001000,-0.000100,0.000000,-0.000100,0.050003,0.000500,0.000000,0.002000,0.000500,0.100000,
0.001000,0.000000,0.000000,0.001000,-0.000100,0.000000,-0.000100,0.050003,0.000500,0.000000,0.002000,0.000500,0.100000,
0.001000,0.000011,0.000000,0.001000,-0.000099,0.000011,-0.000099,0.050003,0.000500,0.000000,0.002000,0.000500,0.100000,
0.001000,0.000000,0.000000,0.001000,-0.000050,0.000000,-0.000050,0.050003,0.000500,0.000010,0.002000,0.000500,0.100000,
0.001000,0.000000,0.000000,0.001000,-0.000050,0.000000,-0.000050,0.050003,0.000500,0.000000,0.002000,0.000500,0.100000,
0.001000,0.000000,0.000000,0.001000,-0.000050,0.000000,-0.000050,0.050003,0.000500,0.000000,0.002000,0.000500,0.100000,
0.001000,0.000000,0.000000,0.001000,-0.000050,0.000000,-0.000050,0.050003,0.000500,0.000000,0.002000,0.000500,0.100000,
0.001000,0.000000,0.000000,0.001000,-0.000050,0.000000,-0.000050,0.050003,0.000500,0.000000,0.002000,0.000500,0.100000,
0.001000,0.000000,0.000000,0.001000,-0.000050,0.000000,-0.000050,0.050003,0.000500,0.000000,0.002000,0.000500,0.100000,
0.001000,0.000000,0.000000,0.001000,-0.000050,0.000000,-0.000050,0.050003,0.000500,0.000000,0.002000,0.000500,0.100000,
0.001000,0.000000,0.000000,0.001000,-0.000050,0.000000,-0.000050,0.050003,0.000500,0.000000,0.002000,0.000500,0.100000,
0.001000,0.000000,0.000000,0.001000,-0.000050,0.000000,-0.000050,0.050003,0.000500,0.000000,0.002000,0.000500,0.100000,
0.001000,0.000000,0.000000,0.001000,0.000000,0.000000,0.000000,0.050003,0.000500,0.000010,0.002000,0.000500,0.100000,
0.001000,0.000000,0.000000,0.001000,0.000000,0.000000,0.000000,0.050003,0.000500,0.000000,0.002000,0.000500,0.100000,
0.001000,0.000000,0.000000,0.001000,0.000000,0.000000,0.000000,0.050003,0.000500,0.000000,0.002000,0.000500,0.100000,
0.001000,0.000000,0.000000,0.001000,0.000000,0.000000,0.000000,0.050003,0.000500,0.000000,0.002000,0.000500,0.100000,
0.001000,0.000000,0.000000,0.001000,0.000000,0.000000,0.000000,0.050003,0.000500,0.000000,0.002000,0.000500,0.100000,
0.001000,0.000000,0.000000,0.001000,0.000000,0.000000,0.000000,0.050003,0.000500,0.000000,0.002000,0.000500,0.100000,
0.001000,0.000000,0.000000,0.001000,0.000000,0.000000,0.000000,0.050003,0.000500,0.000000,0.002000,0.000500,0.100000,
0.001000,0.000000,0.000000,0.001000,0.000000,0.000000,0.000000,0.050003,0.000500,0.000000,0.002000,0.000500,0.100000,
0.001000,0.000000,0.000000,0.001000,0.000000,0.000000,0.000000,0.050003,0.000500,0.000000,0.002000,0.000500,0.100000,
0.001000,0.000000,0.000000,0.001000,0.000050,0.000000,0.000050,0.050003,0.000500,0.000010,0.002000,0.000500,0.100000,
0.001000,0.000000,0.000000,0.001000,0.000050,0.000000,0.000050,0.050003,0.000500,0.000000,0.002000,0.000500,0.100000,
0.001000,0.000000,0.000000,0.001000,0.000050,0.000000,0.000050,0.050003,0.000500,0.000000,0.002000,0.000500,0.100000,
0.001000,0.000000,0.000000,0.001000,0.000050,0.000000,0.000050,0.050003,0.000500,0.000000,0.002000,0.000500,0.100000,
0.001000,0.000000,0.000000,0.001000,0.000050,0.000000,0.000050,0.050003,0.000500,0.000000,0.002000,0.000500,0.100000,
0.001000,0.000000,0.000000,0.001000,0.000050,0.000000,0.000050,0.050003,0.000500,0.000000,0.002000,0.000500,0.100000,
0.001000,0.000000,0.000000,0.001000,0.000050,0.000000,0.000050,0.050003,0.000500,0.000000,0.002000,0.000500,0.100000,
0.001000,0.000000,0.000000,0.001000,0.000050,0.000000,0.000050,0.050003,0.000500,0.000000,0.002000,0.000500,0.100000,
0.001000,0.000000,0.000000,0.001000,0.000050,0.000000,0.000050,0.050003,0.000500,0.000000,0.002000,0.000500,0.100000,
0.001000,0.000000,0.000000,0.001000,0.000100,0.000000,0.000100,0.050003,0.000500,0.000010,0.002000,0.000500,0.100000,
0.001000,0.000000,0.000000,0.001000,0.000100,0.000000,0.000100,0.050003,0.000500,0.000000,0.002000,0.000500,0.100000,
0.001000,0.000000,0.000000,0.001000,0.000100,0.000000,0.000100,0.050003,0.000500,0.000000,0.002000,0.000500,0.100000,
0.001000,0.000000,0.000000,0.001000,0.000100,0.000000,0.000100,0.050003,0.000500,0.000000,0.002000,0.000500,0.100000,
0.001000,0.000000,0.000000,0.001000,0.000100,0.000000,0.000100,0.050003,0.000500,0.000000,0.002000,0.000500,0.100000,
0.001000,0.000000,0.000000,0.001000,0.000100,0.000000,0.000100,0.050003,0.000500,0.000000,0.002000,0.000500,0.100000,
0.001000,0.000000,0.000000,0.001000,0.000100,0.000000,0.000100,0.050003,0.000500,0.000000,0.002000,0.000500,0.100000,
0.001000,0.000000,0.000000,0.001000,0.000100,0.000000,0.000100,0.050003,0.000500,0.000000,0.002000,0.000500,0.100000,
0.001000,-0.000011,0.000000,0.001000,0.000099,-0.000011,0.000099,0.050003,0.000500,0.000000,0.002000,0.000500,0.100000,
0.001000,0.000000,0.000010,0.001001,0.000150,0.000000,0.000150,0.050003,0.000500,0.000010,0.002000,0.000500,0.100000,
0.001000,0.000000,0.000000,0.001001,0.000150,0.000000,0.000150,0.050003,0.000500,0.000000,0.002000,0.000500,0.100000,
0.001000,0.000000,0.000000,0.001001,0.000150,0.000000,0.000150,0.050003,0.000500,0.000000,0.002000,0.000500,0.100000,
0.001000,0.000000,0.000000,0.001001,0.000150,0.000000,0.000150,0.050003,0.000500,0.000000,0.002000,0.000500,0.100000,
0.001000,0.000000,0.000000,0.001001,0.000150,0.000000,0.000150,0.050003,0.000500,0.000000,0.002000,0.000500,0.100000,
0.001000,-0.000010,0.000000,0.001001,0.000150,-0.000010,0.000150,0.050003,0.000500,0.000000,0.002000,0.000500,0.100000,
0.001000,-0.000012,0.000000,0.001001,0.000150,-0.000012,0.000150,0.050003,0.000500,0.000000,0.002000,0.000500,0.100000,
0.001000,-0.000014,0.000000,0.001001,0.000149,-0.000014,0.000149,0.050003,0.000500,0.000000,0.002000,0.000500,0.100000,
0.001000,-0.000016,0.000000,0.001001,0.000149,-0.000016,0.000149,0.050003,0.000500,0.000000,0.002000,0.000500,0.100000,
0.001000,0.000000,0.000010,0.001001,0.000200,0.000000,0.000200,0.050003,0.000500,0.000010,0.002000,0.000500,0.100000,
0.001000,0.000000,0.000000,0.001001,0.000200,0.000000,0.000200,0.050003,0.000500,0.000000,0.002000,0.000500,0.100000,
0.001000,0.000000,0.000000,0.001001,0.000200,0.000000,0.000200,0.050003,0.000500,0.000000,0.002000,0.000500,0.100000,
0.001000,0.000000,0.000000,0.001001,0.000200,0.000000,0.000200,0.050003,0.000500,0.000000,0.002000,0.000500,0.100000,
0.001000,-0.000011,0.000000,0.001001,0.000200,-0.000011,0.000200,0.050003,0.000500,0.000000,0.002000,0.000500,0.100000,
0.001000,-0.000014,0.000000,0.001001,0.000200,-0.000014,0.000200,0.050003,0.000500,0.000000,0.002000,0.000500,0.100000,
0.001000,-0.000016,0.000000,0.001001,0.000199,-0.000016,0.000199,0.050003,0.000500,0.000000,0.002000,0.000500,0.100000,
0.001000,-0.000019,0.000000,0.001001,0.000199,-0.000019,0.000199,0.050003,0.000500,0.000000,0.002000,0.000500,0.100000,
0.001000,-0.000022,0.000000,0.001001,0.000199,-0.000022,0.000199,0.050003,0.000500,0.000000,0.002000,0.000500,0.100000,
0.001000,0.000000,0.000010,0.001002,0.000250,0.000000,0.000250,0.050003,0.000500,0.000010,0.002000,0.000500,0.100000,
0.001000,0.000000,0.000000,0.001002,0.000250,0.000000,0.000250,0.050003,0.000500,0.000000,0.002000,0.000500,0.100000,
0.001000,0.000000,0.000000,0.001002,0.000250,0.000000,0.000250,0.050003,0.000500,0.000000,0.002000,0.000500,0.100000,
0.001000,-0.000010,0.000000,0.001002,0.000250,-0.000010,0.000250,0.050003,0.000500,0.000000,0.002000,0.000500,0.100000,
0.001000,-0.000014,0.000000,0.001002,0.000250,-0.000014,0.000250,0.050003,0.000500,0.000000,0.002000,0.000500,0.100000,
0.001000,-0.000017,0.000000,0.001002,0.000249,-0.000017,0.000249,0.050003,0.000500,0.000000,0.002000,0.000500,0.100000,
0.001000,-0.000021,0.000000,0.001002,0.000249,-0.000021,0.000249,0.050003,0.000500,0.000000,0.002000,0.000500,0.100000,
0.001000,-0.000024,0.000000,0.001002,0.000249,-0.000024,0.000249,0.050003,0.000500,0.000000,0.002000,0.000500,0.100000,
0.001000,-0.000027,0.000000,0.001002,0.000249,-0.000027,0.000249,0.050003,0.000500,0.000000,0.002000,0.000500,0.100000,
0.001000,0.000000,0.000010,0.001002,0.000300,0.000000,0.000300,0.050003,0.000500,0.000010,0.002000,0.000500,0.100000,
0.001000,0.000000,0.000000,0.001002,0.000300,0.000000,0.000300,0.050003,0.000500,0.000000,0.002000,0.000500,0.100000,
0.001000,0.000000,0.000000,0.001002,0.000300,0.000000,0.000300,0.050003,0.000500,0.000000,0.002000,0.000500,0.100000,
0.001000,-0.000012,0.000000,0.001002,0.000300,-0.000012,0.000300,0.050003,0.000500,0.000000,0.002000,0.000500,0.100000,
0.001000,-0.000016,0.000000,0.001002,0.000300,-0.000016,0.000300,0.050003,0.000500,0.000000,0.002000,0.000500,0.100000,
0.001000,-0.000021,0.000000,0.001002,0.000299,-0.000021,0.000299,0.050003,0.000500,0.000000,0.002000,0.000500,0.100000,
0.001000,-0.000025,0.000000,0.001002,0.000299,-0.000025,0.000299,0.050003,0.000500,0.000000,0.002000,0.000500,0.100000,
0.001000,-0.000029,0.000000,0.001002,0.000299,-0.000029,0.000299,0.050003,0.000500,0.000000,0.002000,0.000500,0.100000,
0.001000,-0.000033,0.000000,0.001002,0.000298,-0.000033,0.000298,0.050003,0.000500,0.000000,0.002000,0.000500,0.100000,
0.001000,0.000000,0.000010,0.001003,0.000350,0.000000,0.000350,0.050003,0.000500,0.000010,0.002000,0.000500,0.100000,
0.001000,0.000000,0.000000,0.001003,0.000350,0.000000,0.000350,0.050003,0.000500,0.000000,0.002000,0.000500,0.100000,
0.001000,0.000000,0.000000,0.001003,0.000350,0.000000,0.000350,0.050003,0.000500,0.000000,0.002000,0.000500,0.100000,
0.001000,-0.000014,0.000000,0.001003,0.000350,-0.000014,0.000350,0.050003,0.000500,0.000000,0.002000,0.000500,0.100000,
0.001000,-0.000019,0.000000,0.001003,0.000349,-0.000019,0.000349,0.050003,0.000500,0.000000,0.002000,0.000500,0.100000,
0.001000,-0.000024,0.000000,0.001003,0.000349,-0.000024,0.000349,0.050003,0.000500,0.000000,0.002000,0.000500,0.100000,
0.001000,-0.000029,0.000000,0.001003,0.000349,-0.000029,0.000349,0.050003,0.000500,0.000000,0.002000,0.000500,0.100000,
0.001000,-0.000034,0.000000,0.001003,0.000348,-0.000034,0.000348,0.050003,0.000500,0.000000,0.002000,0.000500,0.100000,
0.001000,-0.000038,0.000000,0.001003,0.000348,-0.000038,0.000348,0.050003,0.000500,0.000000,0.002000,0.000500,0.100000,
0.001000,0.000000,0.000010,0.001004,0.000400,0.000000,0.000400,0.050003,0.000500,0.000010,0.002000,0.000500,0.100000,
0.001000,0.000000,0.000000,0.001004,0.000400,0.000000,0.000400,0.050003,0.000500,0.000000,0.002000,0.000500,0.100000,
0.001000,-0.000011,0.000000,0.001004,0.000400,-0.000011,0.000400,0.050003,0.000500,0.000000,0.002000,0.000500,0.100000,
0.001000,-0.000016,0.000000,0.001004,0.000400,-0.000016,0.000400,0.050003,0.000500,0.000000,0.002000,0.000500,0.100000,
0.001000,-0.000022,0.000000,0.001004,0.000399,-0.000022,0.000399,0.050003,0.000500,0.000000,0.002000,0.000500,0.100000,
0.001000,-0.000027,0.000000,0.001004,0.000399,-0.000027,0.000399,0.050003,0.000500,0.000000,0.002000,0.000500,0.100000,
0.001000,-0.000033,0.000000,0.001004,0.000399,-0.000033,0.000399,0.050003,0.000500,0.000000,0.002000,0.000500,0.100000,
0.001000,-0.000038,0.000000,0.001004,0.000398,-0.000038,0.000398,0.050003,0.000500,0.000000,0.002000,0.000500,0.100000,
0.001000,-0.000044,0.000000,0.001004,0.000398,-0.000044,0.000398,0.050003,0.000500,0.000000,0.002000,0.000500,0.100000,
0.001000,0.000000,0.000010,0.001005,0.000450,0.000000,0.000450,0.050003,0.000500,0.000010,0.002000,0.000500,0.100000,
0.001000,0.000000,0.000000,0.001005,0.000450,0.000000,0.000450,0.050003,0.000500,0.000000,0.002000,0.000500,0.100000,
0.001000,-0.000012,0.000000,0.001005,0.000450,-0.000012,0.000450,0.050003,0.000500,0.000000,0.002000,0.000500,0.100000,
0.001000,-0.000019,0.000000,0.001005,0.000450,-0.000019,0.000450,0.050003,0.000500,0.000000,0.002000,0.000500,0.100000,
0.001000,-0.000025,0.000000,0.001005,0.000449,-0.000025,0.000449,0.050003,0.000500,0.000000,0.002000,0.000500,0.100000,
0.001000,-0.000031,0.000000,0.001005,0.000449,-0.000031,0.000449,0.050003,0.000500,0.000000,0.002000,0.000500,0.100000,
0.001000,-0.000037,0.000000,0.001005,0.000449,-0.000037,0.000449,0.050003,0.000500,0.000000,0.002000,0.000500,0.100000,
0.001000,-0.000043,0.000000,0.001005,0.000448,-0.000043,0.000448,0.050003,0.000500,0.000000,0.002000,0.000500,0.100000,
0.001000,-0.000049,0.000000,0.001005,0.000447,-0.000049,0.000447,0.050003,0.000500,0.000000,0.002000,0.000500,0.100000,
0.001000,0.000000,0.000010,0.001007,0.000500,0.000000,0.000500,0.050003,0.000500,0.000010,0.002000,0.000500,0.100000,
0.001000,0.000000,0.000000,0.001007,0.000500,0.000000,0.000500,0.050003,0.000500,0.000000,0.002000,0.000500,0.100000,
0.001000,-0.000014,0.000000,0.001007,0.000500,-0.000014,0.000500,0.050003,0.000500,0.000000,0.002000,0.000500,0.100000,
0.001000,-0.000021,0.000000,0.001007,0.000500,-0.000021,0.000500,0.050003,0.000500,0.000000,0.002000,0.000500,0.100000,
0.001000,-0.000027,0.000000,0.001007,0.000499,-0.000027,0.000499,0.050003,0.000500,0.000000,0.002000,0.000500,0.100000,
0.001000,-0.000034,0.000000,0.001007,0.000499,-0.000034,0.000499,0.050003,0.000500,0.000000,0.002000,0.000500,0.100000,
0.001000,-0.000041,0.000000,0.001007,0.000498,-0.000041,0.000498,0.050003,0.000500,0.000000,0.002000,0.000500,0.100000,
0.001000,-0.000048,0.000000,0.001007,0.000498,-0.000048,0.000498,0.050003,0.000500,0.000000,0.002000,0.000500,0.100000,
0.001000,-0.000055,0.000000,0.001007,0.000497,-0.000055,0.000497,0.050003,0.000500,0.000000,0.002000,0.000500,0.100000};*/
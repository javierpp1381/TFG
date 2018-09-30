// ==============================================================
// RTL generated by Vivado(TM) HLS - High-Level Synthesis from C, C++ and SystemC
// Version: 2018.2.1
// Copyright (C) 1986-2018 Xilinx, Inc. All Rights Reserved.
// 
// ===========================================================

#ifndef _compute_HH_
#define _compute_HH_

#include "systemc.h"
#include "AESL_pkg.h"

#include "compute_faddfsub_bkb.h"
#include "compute_fadd_32nscud.h"
#include "compute_fmul_32nsdEe.h"
#include "compute_fdiv_32nseOg.h"
#include "compute_accu.h"

namespace ap_rtl {

struct compute : public sc_module {
    // Port declarations 35
    sc_in_clk ap_clk;
    sc_in< sc_logic > ap_rst;
    sc_in< sc_logic > ap_start;
    sc_out< sc_logic > ap_done;
    sc_out< sc_logic > ap_idle;
    sc_out< sc_logic > ap_ready;
    sc_out< sc_lv<4> > covariance_matrix_address0;
    sc_out< sc_logic > covariance_matrix_ce0;
    sc_out< sc_logic > covariance_matrix_we0;
    sc_out< sc_lv<32> > covariance_matrix_d0;
    sc_out< sc_lv<4> > covariance_matrix_address1;
    sc_out< sc_logic > covariance_matrix_ce1;
    sc_out< sc_logic > covariance_matrix_we1;
    sc_out< sc_lv<32> > covariance_matrix_d1;
    sc_out< sc_lv<2> > centroid_address0;
    sc_out< sc_logic > centroid_ce0;
    sc_out< sc_logic > centroid_we0;
    sc_out< sc_lv<32> > centroid_d0;
    sc_out< sc_lv<2> > centroid_address1;
    sc_out< sc_logic > centroid_ce1;
    sc_out< sc_logic > centroid_we1;
    sc_out< sc_lv<32> > centroid_d1;
    sc_out< sc_lv<5> > cloud_address0;
    sc_out< sc_logic > cloud_ce0;
    sc_in< sc_lv<32> > cloud_q0;
    sc_out< sc_lv<5> > cloud_address1;
    sc_out< sc_logic > cloud_ce1;
    sc_in< sc_lv<32> > cloud_q1;
    sc_out< sc_lv<3> > indices_address0;
    sc_out< sc_logic > indices_ce0;
    sc_in< sc_lv<32> > indices_q0;
    sc_out< sc_lv<3> > indices_address1;
    sc_out< sc_logic > indices_ce1;
    sc_in< sc_lv<32> > indices_q1;
    sc_out< sc_lv<32> > ap_return;
    sc_signal< sc_logic > ap_var_for_const0;
    sc_signal< sc_lv<32> > ap_var_for_const1;


    // Module declarations
    compute(sc_module_name name);
    SC_HAS_PROCESS(compute);

    ~compute();

    sc_trace_file* mVcdFile;

    ofstream mHdltvinHandle;
    ofstream mHdltvoutHandle;
    compute_accu* accu_U;
    compute_faddfsub_bkb<1,5,32,32,32>* compute_faddfsub_bkb_U1;
    compute_faddfsub_bkb<1,5,32,32,32>* compute_faddfsub_bkb_U2;
    compute_fadd_32nscud<1,5,32,32,32>* compute_fadd_32nscud_U3;
    compute_fmul_32nsdEe<1,4,32,32,32>* compute_fmul_32nsdEe_U4;
    compute_fmul_32nsdEe<1,4,32,32,32>* compute_fmul_32nsdEe_U5;
    compute_fdiv_32nseOg<1,16,32,32,32>* compute_fdiv_32nseOg_U6;
    sc_signal< sc_lv<50> > ap_CS_fsm;
    sc_signal< sc_logic > ap_CS_fsm_state1;
    sc_signal< sc_lv<32> > reg_620;
    sc_signal< sc_logic > ap_CS_fsm_state5;
    sc_signal< sc_logic > ap_CS_fsm_state8;
    sc_signal< sc_logic > ap_CS_fsm_state11;
    sc_signal< sc_lv<32> > reg_626;
    sc_signal< sc_lv<32> > reg_632;
    sc_signal< sc_logic > ap_CS_fsm_state6;
    sc_signal< sc_logic > ap_CS_fsm_state9;
    sc_signal< sc_logic > ap_CS_fsm_state12;
    sc_signal< sc_logic > ap_CS_fsm_state7;
    sc_signal< sc_logic > ap_CS_fsm_state10;
    sc_signal< sc_lv<32> > grp_fu_599_p2;
    sc_signal< sc_lv<32> > reg_653;
    sc_signal< sc_logic > ap_CS_fsm_state13;
    sc_signal< sc_logic > ap_CS_fsm_state40;
    sc_signal< sc_lv<32> > reg_658;
    sc_signal< sc_logic > ap_CS_fsm_state41;
    sc_signal< sc_lv<32> > reg_663;
    sc_signal< sc_logic > ap_CS_fsm_state42;
    sc_signal< sc_lv<32> > grp_fu_578_p2;
    sc_signal< sc_lv<32> > reg_668;
    sc_signal< sc_logic > ap_CS_fsm_state45;
    sc_signal< sc_logic > ap_CS_fsm_state46;
    sc_signal< sc_lv<32> > reg_676;
    sc_signal< sc_logic > ap_CS_fsm_state14;
    sc_signal< sc_logic > ap_CS_fsm_state47;
    sc_signal< sc_lv<32> > grp_fu_586_p2;
    sc_signal< sc_lv<32> > reg_683;
    sc_signal< sc_logic > ap_CS_fsm_state16;
    sc_signal< sc_lv<32> > reg_691;
    sc_signal< sc_logic > ap_CS_fsm_state17;
    sc_signal< sc_lv<32> > accu_q1;
    sc_signal< sc_lv<32> > reg_698;
    sc_signal< sc_logic > ap_CS_fsm_state21;
    sc_signal< sc_logic > ap_CS_fsm_state37;
    sc_signal< sc_lv<32> > accu_q0;
    sc_signal< sc_lv<32> > reg_707;
    sc_signal< sc_lv<32> > reg_717;
    sc_signal< sc_logic > ap_CS_fsm_state38;
    sc_signal< sc_lv<4> > indvarinc_fu_727_p2;
    sc_signal< sc_logic > ap_CS_fsm_state2;
    sc_signal< sc_lv<1> > tmp_1_fu_738_p2;
    sc_signal< sc_lv<8> > i_1_fu_750_p2;
    sc_signal< sc_lv<8> > i_1_reg_1355;
    sc_signal< sc_logic > ap_CS_fsm_state3;
    sc_signal< sc_lv<3> > indices_addr_reg_1360;
    sc_signal< sc_lv<1> > exitcond1_fu_744_p2;
    sc_signal< sc_logic > ap_CS_fsm_state4;
    sc_signal< sc_lv<32> > tmp_25_reg_1436;
    sc_signal< sc_lv<32> > tmp_29_reg_1446;
    sc_signal< sc_lv<32> > tmp_22_reg_1451;
    sc_signal< sc_logic > ap_CS_fsm_state15;
    sc_signal< sc_lv<32> > tmp_26_reg_1457;
    sc_signal< sc_lv<32> > grp_fu_591_p2;
    sc_signal< sc_lv<32> > tmp_38_reg_1463;
    sc_signal< sc_lv<32> > tmp_30_reg_1469;
    sc_signal< sc_lv<32> > tmp_34_reg_1475;
    sc_signal< sc_logic > ap_CS_fsm_state18;
    sc_signal< sc_lv<4> > i_2_fu_1232_p2;
    sc_signal< sc_lv<4> > i_2_reg_1484;
    sc_signal< sc_logic > ap_CS_fsm_state20;
    sc_signal< sc_lv<4> > accu_addr_10_reg_1489;
    sc_signal< sc_lv<1> > exitcond_fu_1226_p2;
    sc_signal< sc_lv<32> > accu_load_3_reg_1494;
    sc_signal< sc_lv<32> > accu_load_4_reg_1499;
    sc_signal< sc_lv<32> > grp_fu_607_p2;
    sc_signal< sc_lv<32> > tmp_43_reg_1504;
    sc_signal< sc_lv<32> > accu_load_6_reg_1509;
    sc_signal< sc_lv<32> > tmp_47_reg_1514;
    sc_signal< sc_lv<32> > tmp_51_reg_1519;
    sc_signal< sc_lv<4> > accu_address0;
    sc_signal< sc_logic > accu_ce0;
    sc_signal< sc_logic > accu_we0;
    sc_signal< sc_lv<32> > accu_d0;
    sc_signal< sc_lv<4> > accu_address1;
    sc_signal< sc_logic > accu_ce1;
    sc_signal< sc_logic > accu_we1;
    sc_signal< sc_lv<32> > accu_d1;
    sc_signal< sc_lv<4> > invdar_reg_437;
    sc_signal< sc_lv<32> > tmp_2_reg_448;
    sc_signal< sc_logic > ap_CS_fsm_state19;
    sc_signal< sc_lv<32> > tmp_3_reg_460;
    sc_signal< sc_lv<32> > tmp_4_reg_472;
    sc_signal< sc_lv<32> > tmp_5_reg_484;
    sc_signal< sc_lv<32> > tmp_6_reg_496;
    sc_signal< sc_lv<32> > tmp_7_reg_508;
    sc_signal< sc_lv<32> > tmp_8_reg_520;
    sc_signal< sc_lv<32> > tmp_9_reg_532;
    sc_signal< sc_lv<32> > tmp_s_reg_544;
    sc_signal< sc_lv<8> > i_reg_556;
    sc_signal< sc_lv<4> > i1_reg_567;
    sc_signal< sc_logic > ap_CS_fsm_state36;
    sc_signal< sc_lv<64> > tmp_fu_733_p1;
    sc_signal< sc_lv<64> > tmp_10_fu_756_p1;
    sc_signal< sc_lv<64> > tmp_56_cast_fu_783_p1;
    sc_signal< sc_lv<64> > tmp_58_cast_fu_810_p1;
    sc_signal< sc_lv<64> > tmp_60_cast_fu_837_p1;
    sc_signal< sc_lv<64> > tmp_63_cast_fu_870_p1;
    sc_signal< sc_lv<64> > tmp_65_cast_fu_897_p1;
    sc_signal< sc_lv<64> > tmp_68_cast_fu_930_p1;
    sc_signal< sc_lv<64> > tmp_71_cast_fu_963_p1;
    sc_signal< sc_lv<64> > tmp_74_cast_fu_996_p1;
    sc_signal< sc_lv<64> > tmp_77_cast_fu_1029_p1;
    sc_signal< sc_lv<64> > tmp_80_cast_fu_1062_p1;
    sc_signal< sc_lv<64> > tmp_83_cast_fu_1095_p1;
    sc_signal< sc_lv<64> > tmp_86_cast_fu_1128_p1;
    sc_signal< sc_lv<64> > tmp_88_cast_fu_1155_p1;
    sc_signal< sc_lv<64> > tmp_91_cast_fu_1188_p1;
    sc_signal< sc_lv<64> > tmp_94_cast_fu_1221_p1;
    sc_signal< sc_lv<64> > tmp_53_fu_1238_p1;
    sc_signal< sc_logic > ap_CS_fsm_state39;
    sc_signal< sc_lv<32> > grp_fu_613_p2;
    sc_signal< sc_logic > ap_CS_fsm_state48;
    sc_signal< sc_logic > ap_CS_fsm_state49;
    sc_signal< sc_logic > ap_CS_fsm_state50;
    sc_signal< sc_lv<32> > grp_fu_578_p0;
    sc_signal< sc_lv<32> > grp_fu_578_p1;
    sc_signal< sc_logic > ap_CS_fsm_state43;
    sc_signal< sc_lv<32> > grp_fu_586_p0;
    sc_signal< sc_lv<32> > grp_fu_586_p1;
    sc_signal< sc_lv<32> > grp_fu_599_p0;
    sc_signal< sc_lv<32> > grp_fu_599_p1;
    sc_signal< sc_lv<32> > grp_fu_607_p0;
    sc_signal< sc_lv<32> > grp_fu_607_p1;
    sc_signal< sc_lv<4> > tmp_12_fu_765_p1;
    sc_signal< sc_lv<6> > p_shl14_cast_fu_769_p3;
    sc_signal< sc_lv<6> > tmp_11_fu_761_p1;
    sc_signal< sc_lv<6> > tmp_15_fu_777_p2;
    sc_signal< sc_lv<4> > tmp_19_fu_792_p1;
    sc_signal< sc_lv<6> > p_shl13_cast_fu_796_p3;
    sc_signal< sc_lv<6> > tmp_16_fu_788_p1;
    sc_signal< sc_lv<6> > tmp_20_fu_804_p2;
    sc_signal< sc_lv<4> > tmp_24_fu_819_p1;
    sc_signal< sc_lv<6> > p_shl12_cast_fu_823_p3;
    sc_signal< sc_lv<6> > tmp_23_fu_815_p1;
    sc_signal< sc_lv<6> > tmp_27_fu_831_p2;
    sc_signal< sc_lv<4> > tmp_31_fu_846_p1;
    sc_signal< sc_lv<6> > p_shl11_cast_fu_850_p3;
    sc_signal< sc_lv<6> > tmp_28_fu_842_p1;
    sc_signal< sc_lv<6> > tmp_32_fu_858_p2;
    sc_signal< sc_lv<6> > tmp_35_fu_864_p2;
    sc_signal< sc_lv<4> > tmp_39_fu_879_p1;
    sc_signal< sc_lv<6> > p_shl10_cast_fu_883_p3;
    sc_signal< sc_lv<6> > tmp_37_fu_875_p1;
    sc_signal< sc_lv<6> > tmp_55_fu_891_p2;
    sc_signal< sc_lv<4> > tmp_57_fu_906_p1;
    sc_signal< sc_lv<6> > p_shl9_cast_fu_910_p3;
    sc_signal< sc_lv<6> > tmp_56_fu_902_p1;
    sc_signal< sc_lv<6> > tmp_58_fu_918_p2;
    sc_signal< sc_lv<6> > tmp_59_fu_924_p2;
    sc_signal< sc_lv<4> > tmp_61_fu_939_p1;
    sc_signal< sc_lv<6> > p_shl8_cast_fu_943_p3;
    sc_signal< sc_lv<6> > tmp_60_fu_935_p1;
    sc_signal< sc_lv<6> > tmp_62_fu_951_p2;
    sc_signal< sc_lv<6> > tmp_63_fu_957_p2;
    sc_signal< sc_lv<4> > tmp_65_fu_972_p1;
    sc_signal< sc_lv<6> > p_shl7_cast_fu_976_p3;
    sc_signal< sc_lv<6> > tmp_64_fu_968_p1;
    sc_signal< sc_lv<6> > tmp_66_fu_984_p2;
    sc_signal< sc_lv<6> > tmp_67_fu_990_p2;
    sc_signal< sc_lv<4> > tmp_69_fu_1005_p1;
    sc_signal< sc_lv<6> > p_shl6_cast_fu_1009_p3;
    sc_signal< sc_lv<6> > tmp_68_fu_1001_p1;
    sc_signal< sc_lv<6> > tmp_70_fu_1017_p2;
    sc_signal< sc_lv<6> > tmp_71_fu_1023_p2;
    sc_signal< sc_lv<4> > tmp_73_fu_1038_p1;
    sc_signal< sc_lv<6> > p_shl5_cast_fu_1042_p3;
    sc_signal< sc_lv<6> > tmp_72_fu_1034_p1;
    sc_signal< sc_lv<6> > tmp_74_fu_1050_p2;
    sc_signal< sc_lv<6> > tmp_75_fu_1056_p2;
    sc_signal< sc_lv<4> > tmp_77_fu_1071_p1;
    sc_signal< sc_lv<6> > p_shl4_cast_fu_1075_p3;
    sc_signal< sc_lv<6> > tmp_76_fu_1067_p1;
    sc_signal< sc_lv<6> > tmp_78_fu_1083_p2;
    sc_signal< sc_lv<6> > tmp_79_fu_1089_p2;
    sc_signal< sc_lv<4> > tmp_81_fu_1104_p1;
    sc_signal< sc_lv<6> > p_shl3_cast_fu_1108_p3;
    sc_signal< sc_lv<6> > tmp_80_fu_1100_p1;
    sc_signal< sc_lv<6> > tmp_82_fu_1116_p2;
    sc_signal< sc_lv<6> > tmp_83_fu_1122_p2;
    sc_signal< sc_lv<4> > tmp_85_fu_1137_p1;
    sc_signal< sc_lv<6> > p_shl2_cast_fu_1141_p3;
    sc_signal< sc_lv<6> > tmp_84_fu_1133_p1;
    sc_signal< sc_lv<6> > tmp_86_fu_1149_p2;
    sc_signal< sc_lv<4> > tmp_88_fu_1164_p1;
    sc_signal< sc_lv<6> > p_shl1_cast_fu_1168_p3;
    sc_signal< sc_lv<6> > tmp_87_fu_1160_p1;
    sc_signal< sc_lv<6> > tmp_89_fu_1176_p2;
    sc_signal< sc_lv<6> > tmp_90_fu_1182_p2;
    sc_signal< sc_lv<4> > tmp_92_fu_1197_p1;
    sc_signal< sc_lv<6> > p_shl_cast_fu_1201_p3;
    sc_signal< sc_lv<6> > tmp_91_fu_1193_p1;
    sc_signal< sc_lv<6> > tmp_93_fu_1209_p2;
    sc_signal< sc_lv<6> > tmp_94_fu_1215_p2;
    sc_signal< sc_lv<2> > grp_fu_578_opcode;
    sc_signal< sc_lv<2> > grp_fu_586_opcode;
    sc_signal< sc_lv<50> > ap_NS_fsm;
    static const sc_logic ap_const_logic_1;
    static const sc_logic ap_const_logic_0;
    static const sc_lv<50> ap_ST_fsm_state1;
    static const sc_lv<50> ap_ST_fsm_state2;
    static const sc_lv<50> ap_ST_fsm_state3;
    static const sc_lv<50> ap_ST_fsm_state4;
    static const sc_lv<50> ap_ST_fsm_state5;
    static const sc_lv<50> ap_ST_fsm_state6;
    static const sc_lv<50> ap_ST_fsm_state7;
    static const sc_lv<50> ap_ST_fsm_state8;
    static const sc_lv<50> ap_ST_fsm_state9;
    static const sc_lv<50> ap_ST_fsm_state10;
    static const sc_lv<50> ap_ST_fsm_state11;
    static const sc_lv<50> ap_ST_fsm_state12;
    static const sc_lv<50> ap_ST_fsm_state13;
    static const sc_lv<50> ap_ST_fsm_state14;
    static const sc_lv<50> ap_ST_fsm_state15;
    static const sc_lv<50> ap_ST_fsm_state16;
    static const sc_lv<50> ap_ST_fsm_state17;
    static const sc_lv<50> ap_ST_fsm_state18;
    static const sc_lv<50> ap_ST_fsm_state19;
    static const sc_lv<50> ap_ST_fsm_state20;
    static const sc_lv<50> ap_ST_fsm_state21;
    static const sc_lv<50> ap_ST_fsm_state22;
    static const sc_lv<50> ap_ST_fsm_state23;
    static const sc_lv<50> ap_ST_fsm_state24;
    static const sc_lv<50> ap_ST_fsm_state25;
    static const sc_lv<50> ap_ST_fsm_state26;
    static const sc_lv<50> ap_ST_fsm_state27;
    static const sc_lv<50> ap_ST_fsm_state28;
    static const sc_lv<50> ap_ST_fsm_state29;
    static const sc_lv<50> ap_ST_fsm_state30;
    static const sc_lv<50> ap_ST_fsm_state31;
    static const sc_lv<50> ap_ST_fsm_state32;
    static const sc_lv<50> ap_ST_fsm_state33;
    static const sc_lv<50> ap_ST_fsm_state34;
    static const sc_lv<50> ap_ST_fsm_state35;
    static const sc_lv<50> ap_ST_fsm_state36;
    static const sc_lv<50> ap_ST_fsm_state37;
    static const sc_lv<50> ap_ST_fsm_state38;
    static const sc_lv<50> ap_ST_fsm_state39;
    static const sc_lv<50> ap_ST_fsm_state40;
    static const sc_lv<50> ap_ST_fsm_state41;
    static const sc_lv<50> ap_ST_fsm_state42;
    static const sc_lv<50> ap_ST_fsm_state43;
    static const sc_lv<50> ap_ST_fsm_state44;
    static const sc_lv<50> ap_ST_fsm_state45;
    static const sc_lv<50> ap_ST_fsm_state46;
    static const sc_lv<50> ap_ST_fsm_state47;
    static const sc_lv<50> ap_ST_fsm_state48;
    static const sc_lv<50> ap_ST_fsm_state49;
    static const sc_lv<50> ap_ST_fsm_state50;
    static const sc_lv<32> ap_const_lv32_0;
    static const sc_lv<32> ap_const_lv32_4;
    static const sc_lv<32> ap_const_lv32_7;
    static const sc_lv<32> ap_const_lv32_A;
    static const sc_lv<32> ap_const_lv32_5;
    static const sc_lv<32> ap_const_lv32_8;
    static const sc_lv<32> ap_const_lv32_B;
    static const sc_lv<32> ap_const_lv32_6;
    static const sc_lv<32> ap_const_lv32_9;
    static const sc_lv<32> ap_const_lv32_C;
    static const sc_lv<32> ap_const_lv32_27;
    static const sc_lv<32> ap_const_lv32_28;
    static const sc_lv<32> ap_const_lv32_29;
    static const sc_lv<32> ap_const_lv32_2C;
    static const sc_lv<32> ap_const_lv32_2D;
    static const sc_lv<32> ap_const_lv32_D;
    static const sc_lv<32> ap_const_lv32_2E;
    static const sc_lv<32> ap_const_lv32_F;
    static const sc_lv<32> ap_const_lv32_10;
    static const sc_lv<32> ap_const_lv32_14;
    static const sc_lv<32> ap_const_lv32_24;
    static const sc_lv<32> ap_const_lv32_25;
    static const sc_lv<32> ap_const_lv32_1;
    static const sc_lv<1> ap_const_lv1_1;
    static const sc_lv<32> ap_const_lv32_2;
    static const sc_lv<1> ap_const_lv1_0;
    static const sc_lv<32> ap_const_lv32_3;
    static const sc_lv<32> ap_const_lv32_E;
    static const sc_lv<32> ap_const_lv32_11;
    static const sc_lv<32> ap_const_lv32_13;
    static const sc_lv<4> ap_const_lv4_0;
    static const sc_lv<32> ap_const_lv32_12;
    static const sc_lv<8> ap_const_lv8_0;
    static const sc_lv<32> ap_const_lv32_23;
    static const sc_lv<64> ap_const_lv64_0;
    static const sc_lv<64> ap_const_lv64_1;
    static const sc_lv<64> ap_const_lv64_2;
    static const sc_lv<64> ap_const_lv64_4;
    static const sc_lv<64> ap_const_lv64_5;
    static const sc_lv<64> ap_const_lv64_8;
    static const sc_lv<64> ap_const_lv64_3;
    static const sc_lv<64> ap_const_lv64_6;
    static const sc_lv<64> ap_const_lv64_7;
    static const sc_lv<32> ap_const_lv32_26;
    static const sc_lv<32> ap_const_lv32_3F800000;
    static const sc_lv<32> ap_const_lv32_2F;
    static const sc_lv<32> ap_const_lv32_30;
    static const sc_lv<32> ap_const_lv32_31;
    static const sc_lv<32> ap_const_lv32_2A;
    static const sc_lv<32> ap_const_lv32_435F0000;
    static const sc_lv<4> ap_const_lv4_1;
    static const sc_lv<4> ap_const_lv4_8;
    static const sc_lv<8> ap_const_lv8_DF;
    static const sc_lv<8> ap_const_lv8_1;
    static const sc_lv<2> ap_const_lv2_0;
    static const sc_lv<6> ap_const_lv6_1;
    static const sc_lv<6> ap_const_lv6_2;
    static const sc_lv<4> ap_const_lv4_9;
    static const sc_lv<2> ap_const_lv2_1;
    static const bool ap_const_boolean_1;
    // Thread declarations
    void thread_ap_var_for_const0();
    void thread_ap_var_for_const1();
    void thread_ap_clk_no_reset_();
    void thread_accu_address0();
    void thread_accu_address1();
    void thread_accu_ce0();
    void thread_accu_ce1();
    void thread_accu_d0();
    void thread_accu_d1();
    void thread_accu_we0();
    void thread_accu_we1();
    void thread_ap_CS_fsm_state1();
    void thread_ap_CS_fsm_state10();
    void thread_ap_CS_fsm_state11();
    void thread_ap_CS_fsm_state12();
    void thread_ap_CS_fsm_state13();
    void thread_ap_CS_fsm_state14();
    void thread_ap_CS_fsm_state15();
    void thread_ap_CS_fsm_state16();
    void thread_ap_CS_fsm_state17();
    void thread_ap_CS_fsm_state18();
    void thread_ap_CS_fsm_state19();
    void thread_ap_CS_fsm_state2();
    void thread_ap_CS_fsm_state20();
    void thread_ap_CS_fsm_state21();
    void thread_ap_CS_fsm_state3();
    void thread_ap_CS_fsm_state36();
    void thread_ap_CS_fsm_state37();
    void thread_ap_CS_fsm_state38();
    void thread_ap_CS_fsm_state39();
    void thread_ap_CS_fsm_state4();
    void thread_ap_CS_fsm_state40();
    void thread_ap_CS_fsm_state41();
    void thread_ap_CS_fsm_state42();
    void thread_ap_CS_fsm_state43();
    void thread_ap_CS_fsm_state45();
    void thread_ap_CS_fsm_state46();
    void thread_ap_CS_fsm_state47();
    void thread_ap_CS_fsm_state48();
    void thread_ap_CS_fsm_state49();
    void thread_ap_CS_fsm_state5();
    void thread_ap_CS_fsm_state50();
    void thread_ap_CS_fsm_state6();
    void thread_ap_CS_fsm_state7();
    void thread_ap_CS_fsm_state8();
    void thread_ap_CS_fsm_state9();
    void thread_ap_done();
    void thread_ap_idle();
    void thread_ap_ready();
    void thread_ap_return();
    void thread_centroid_address0();
    void thread_centroid_address1();
    void thread_centroid_ce0();
    void thread_centroid_ce1();
    void thread_centroid_d0();
    void thread_centroid_d1();
    void thread_centroid_we0();
    void thread_centroid_we1();
    void thread_cloud_address0();
    void thread_cloud_address1();
    void thread_cloud_ce0();
    void thread_cloud_ce1();
    void thread_covariance_matrix_address0();
    void thread_covariance_matrix_address1();
    void thread_covariance_matrix_ce0();
    void thread_covariance_matrix_ce1();
    void thread_covariance_matrix_d0();
    void thread_covariance_matrix_d1();
    void thread_covariance_matrix_we0();
    void thread_covariance_matrix_we1();
    void thread_exitcond1_fu_744_p2();
    void thread_exitcond_fu_1226_p2();
    void thread_grp_fu_578_opcode();
    void thread_grp_fu_578_p0();
    void thread_grp_fu_578_p1();
    void thread_grp_fu_586_opcode();
    void thread_grp_fu_586_p0();
    void thread_grp_fu_586_p1();
    void thread_grp_fu_599_p0();
    void thread_grp_fu_599_p1();
    void thread_grp_fu_607_p0();
    void thread_grp_fu_607_p1();
    void thread_i_1_fu_750_p2();
    void thread_i_2_fu_1232_p2();
    void thread_indices_address0();
    void thread_indices_address1();
    void thread_indices_ce0();
    void thread_indices_ce1();
    void thread_indvarinc_fu_727_p2();
    void thread_p_shl10_cast_fu_883_p3();
    void thread_p_shl11_cast_fu_850_p3();
    void thread_p_shl12_cast_fu_823_p3();
    void thread_p_shl13_cast_fu_796_p3();
    void thread_p_shl14_cast_fu_769_p3();
    void thread_p_shl1_cast_fu_1168_p3();
    void thread_p_shl2_cast_fu_1141_p3();
    void thread_p_shl3_cast_fu_1108_p3();
    void thread_p_shl4_cast_fu_1075_p3();
    void thread_p_shl5_cast_fu_1042_p3();
    void thread_p_shl6_cast_fu_1009_p3();
    void thread_p_shl7_cast_fu_976_p3();
    void thread_p_shl8_cast_fu_943_p3();
    void thread_p_shl9_cast_fu_910_p3();
    void thread_p_shl_cast_fu_1201_p3();
    void thread_tmp_10_fu_756_p1();
    void thread_tmp_11_fu_761_p1();
    void thread_tmp_12_fu_765_p1();
    void thread_tmp_15_fu_777_p2();
    void thread_tmp_16_fu_788_p1();
    void thread_tmp_19_fu_792_p1();
    void thread_tmp_1_fu_738_p2();
    void thread_tmp_20_fu_804_p2();
    void thread_tmp_23_fu_815_p1();
    void thread_tmp_24_fu_819_p1();
    void thread_tmp_27_fu_831_p2();
    void thread_tmp_28_fu_842_p1();
    void thread_tmp_31_fu_846_p1();
    void thread_tmp_32_fu_858_p2();
    void thread_tmp_35_fu_864_p2();
    void thread_tmp_37_fu_875_p1();
    void thread_tmp_39_fu_879_p1();
    void thread_tmp_53_fu_1238_p1();
    void thread_tmp_55_fu_891_p2();
    void thread_tmp_56_cast_fu_783_p1();
    void thread_tmp_56_fu_902_p1();
    void thread_tmp_57_fu_906_p1();
    void thread_tmp_58_cast_fu_810_p1();
    void thread_tmp_58_fu_918_p2();
    void thread_tmp_59_fu_924_p2();
    void thread_tmp_60_cast_fu_837_p1();
    void thread_tmp_60_fu_935_p1();
    void thread_tmp_61_fu_939_p1();
    void thread_tmp_62_fu_951_p2();
    void thread_tmp_63_cast_fu_870_p1();
    void thread_tmp_63_fu_957_p2();
    void thread_tmp_64_fu_968_p1();
    void thread_tmp_65_cast_fu_897_p1();
    void thread_tmp_65_fu_972_p1();
    void thread_tmp_66_fu_984_p2();
    void thread_tmp_67_fu_990_p2();
    void thread_tmp_68_cast_fu_930_p1();
    void thread_tmp_68_fu_1001_p1();
    void thread_tmp_69_fu_1005_p1();
    void thread_tmp_70_fu_1017_p2();
    void thread_tmp_71_cast_fu_963_p1();
    void thread_tmp_71_fu_1023_p2();
    void thread_tmp_72_fu_1034_p1();
    void thread_tmp_73_fu_1038_p1();
    void thread_tmp_74_cast_fu_996_p1();
    void thread_tmp_74_fu_1050_p2();
    void thread_tmp_75_fu_1056_p2();
    void thread_tmp_76_fu_1067_p1();
    void thread_tmp_77_cast_fu_1029_p1();
    void thread_tmp_77_fu_1071_p1();
    void thread_tmp_78_fu_1083_p2();
    void thread_tmp_79_fu_1089_p2();
    void thread_tmp_80_cast_fu_1062_p1();
    void thread_tmp_80_fu_1100_p1();
    void thread_tmp_81_fu_1104_p1();
    void thread_tmp_82_fu_1116_p2();
    void thread_tmp_83_cast_fu_1095_p1();
    void thread_tmp_83_fu_1122_p2();
    void thread_tmp_84_fu_1133_p1();
    void thread_tmp_85_fu_1137_p1();
    void thread_tmp_86_cast_fu_1128_p1();
    void thread_tmp_86_fu_1149_p2();
    void thread_tmp_87_fu_1160_p1();
    void thread_tmp_88_cast_fu_1155_p1();
    void thread_tmp_88_fu_1164_p1();
    void thread_tmp_89_fu_1176_p2();
    void thread_tmp_90_fu_1182_p2();
    void thread_tmp_91_cast_fu_1188_p1();
    void thread_tmp_91_fu_1193_p1();
    void thread_tmp_92_fu_1197_p1();
    void thread_tmp_93_fu_1209_p2();
    void thread_tmp_94_cast_fu_1221_p1();
    void thread_tmp_94_fu_1215_p2();
    void thread_tmp_fu_733_p1();
    void thread_ap_NS_fsm();
    void thread_hdltv_gen();
};

}

using namespace ap_rtl;

#endif

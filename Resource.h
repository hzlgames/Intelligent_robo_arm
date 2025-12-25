//{{NO_DEPENDENCIES}}
// Microsoft Visual C++ 生成的包含文件。
// 由 My.rc 使用
//
#define IDR_MAINFRAME					128
#define IDM_ABOUTBOX					0x0010
#define IDD_ABOUTBOX					100
#define IDS_ABOUTBOX					101
#define IDD_MY_DIALOG				102
#define IDD_PAGE_SERIAL              103
#define IDD_PAGE_CAMERA              104
#define IDD_PAGE_CONTROL             105
#define IDD_PAGE_MOTION              108

#define IDC_BTN_DIAGNOSTICS          1000
#define IDC_BTN_EXPORT_PARAMS        1001
#define IDC_BTN_IMPORT_PARAMS        1002

#define IDC_COMBO_COMPORT            1100
#define IDC_BTN_REFRESH_COM          1101
#define IDC_CHECK_SIMULATE           1102
#define IDC_BTN_SERIAL_CONNECT       1103
#define IDC_STATIC_SERIAL_STATUS     1104
#define IDC_BTN_SERIAL_SEND_MOVE     1105
#define IDC_BTN_SERIAL_SEND_READALL  1106
#define IDC_BTN_SERIAL_CLEARLOG      1107
#define IDC_BTN_SERIAL_EXPORTLOG     1108
#define IDC_EDIT_SERIAL_LOG          1109

// Serial manual move / calibration controls
#define IDC_EDIT_MOVE_ID             1110
#define IDC_EDIT_MOVE_POS            1111
#define IDC_EDIT_MOVE_STEP           1112
#define IDC_EDIT_MOVE_TIME           1113
#define IDC_BTN_MOVE_MINUS           1114
#define IDC_BTN_MOVE_PLUS            1115
#define IDC_BTN_MOVE_SEND            1116
#define IDC_BTN_READ_ID              1117
#define IDC_BTN_SET_MIN              1118
#define IDC_BTN_SET_MAX              1119

// Serial settings management
#define IDC_BTN_SERIAL_SHOW_SETTINGS 1120
#define IDC_BTN_SERIAL_CLEAR_SETTINGS 1121

#define IDD_CAMERA_DIAG_PAGE         106
#define IDC_COMBO_CAMERA             1200
#define IDC_BTN_REFRESH_CAM          1201
#define IDC_BTN_START_CAM            1202
#define IDC_BTN_STOP_CAM             1203
#define IDC_BTN_SCREENSHOT           1204
#define IDC_STATIC_VIDEO             1205
#define IDC_STATIC_CAM_STATUS        1206
#define IDC_STATIC_CAM_INFO          1207
#define IDC_EDIT_CAM_LOG             1208
#define IDC_CHECK_MIRROR             1209
#define IDC_CHECK_CROSSHAIR          1210
#define IDC_CHECK_REFLINES           1211
#define IDC_COMBO_ROTATION           1212

// Camera settings management
#define IDC_BTN_CAM_SHOW_SETTINGS    1213
#define IDC_BTN_CAM_RESET_SETTINGS   1214

// Control/Throttle diagnostics page
#define IDD_THROTTLE_DIAG_PAGE       107
#define IDC_SLIDER_THROTTLE          1300
#define IDC_STATIC_THROTTLE_VALUE    1301
#define IDC_STATIC_SEND_FPS          1302
#define IDC_STATIC_LAST_SEND_TIME    1303
#define IDC_EDIT_THROTTLE_LOG        1304

// Motion diagnostics page
#define IDC_MOTION_COMBO_COMPORT     1400
#define IDC_MOTION_BTN_REFRESH_COM   1401
#define IDC_MOTION_CHECK_SIMULATE    1402
#define IDC_MOTION_BTN_CONNECT       1403
#define IDC_MOTION_STATIC_STATUS     1404

#define IDC_MOTION_COMBO_JOINT       1410
#define IDC_MOTION_EDIT_SERVOID      1411
#define IDC_MOTION_EDIT_MIN          1412
#define IDC_MOTION_EDIT_MAX          1413
#define IDC_MOTION_EDIT_HOME         1414
#define IDC_MOTION_CHECK_INVERT      1415
#define IDC_MOTION_BTN_LOAD_ALL      1416
#define IDC_MOTION_BTN_SAVE_ALL      1417
#define IDC_MOTION_BTN_IMPORT_LIMITS 1418

#define IDC_MOTION_EDIT_TARGET       1420
#define IDC_MOTION_EDIT_TIME         1421
#define IDC_MOTION_BTN_MOVE          1422
#define IDC_MOTION_BTN_HOME          1423
#define IDC_MOTION_BTN_READALL       1424

#define IDC_MOTION_CHECK_LOOP        1430
#define IDC_MOTION_BTN_DEMO_PLAY     1431
#define IDC_MOTION_BTN_STOP          1432

#define IDC_MOTION_EDIT_LOG          1440

// =========================
// 主界面：相机预览 + Jog 控制
// =========================

// Camera preview controls (main dialog)
#define IDC_MAIN_COMBO_CAMERA        1500
#define IDC_MAIN_BTN_REFRESH_CAM     1501
#define IDC_MAIN_BTN_START_CAM       1502
#define IDC_MAIN_BTN_STOP_CAM        1503
#define IDC_MAIN_STATIC_CAM_STATUS   1504
#define IDC_MAIN_STATIC_CAM_INFO     1505
#define IDC_MAIN_STATIC_VIDEO        1506

// Overlay toggles (main dialog)
#define IDC_MAIN_CHECK_MIRROR        1510
#define IDC_MAIN_CHECK_CROSSHAIR     1511
#define IDC_MAIN_CHECK_GRID          1512
#define IDC_MAIN_COMBO_ROTATION      1513

// Jog controls (main dialog)
#define IDC_MAIN_STATIC_JOGPAD       1520
#define IDC_MAIN_SLIDER_SPEED_MM     1521
#define IDC_MAIN_SLIDER_SPEED_PITCH  1522
#define IDC_MAIN_STATIC_POSE         1523
#define IDC_MAIN_BTN_EMERGENCY_STOP  1524

// Main dialog: Serial quick controls (right side, next to camera preview)
#define IDC_MAIN_GROUP_SERIAL        1525
#define IDC_MAIN_COMBO_COMPORT       1526
#define IDC_MAIN_BTN_REFRESH_COM     1527
#define IDC_MAIN_CHECK_SIMULATE      1528
#define IDC_MAIN_BTN_CONNECT         1529
#define IDC_MAIN_STATIC_SERIAL_STATUS 1530

// Main dialog groups (for resize/layout)
#define IDC_MAIN_GROUP_JOG           1531
#define IDC_MAIN_GROUP_STATUS        1532

// Main dialog labels (avoid IDC_STATIC collisions; required for reliable resize layout)
#define IDC_MAIN_LBL_CAMERA          1533
#define IDC_MAIN_LBL_ROTATION        1534
#define IDC_MAIN_LBL_SERIAL_COM      1535
#define IDC_MAIN_LBL_SPEED_MM        1536
#define IDC_MAIN_LBL_PITCH_SPEED     1537

// Main dialog: Visual servo controls (vision-follow)
#define IDC_MAIN_GROUP_VS            1538
#define IDC_MAIN_CHECK_VS_ENABLE     1539
#define IDC_MAIN_LBL_VS_MODE         1540
#define IDC_MAIN_COMBO_VS_MODE       1541
#define IDC_MAIN_LBL_VS_ADVANCE      1542
#define IDC_MAIN_SLIDER_VS_ADVANCE   1543
#define IDC_MAIN_CHECK_VS_OVERRIDE   1544
#define IDC_MAIN_STATIC_VS_STATUS    1545

// String resources
#define IDS_DIAG_TITLE               2000
#define IDS_TAB_SERIAL               2001
#define IDS_TAB_CAMERA               2002
#define IDS_TAB_CONTROL              2003
#define IDS_TAB_MOTION               2012
#define IDS_STATUS_DISCONNECTED      2004
#define IDS_STATUS_CONNECTED_SIM     2005
#define IDS_STATUS_CONNECTED_REAL    2006
#define IDS_MSG_SELECT_COM           2007
#define IDS_MSG_OPEN_COM_FAILED      2008
#define IDS_MSG_EXPORT_FAILED        2009
#define IDS_INFO_CONNECTED_SIM       2010
#define IDS_INFO_DISCONNECTED        2011

// 新对象的下一组默认值
//
#ifdef APSTUDIO_INVOKED
#ifndef APSTUDIO_READONLY_SYMBOLS

#define _APS_NEXT_RESOURCE_VALUE	129
#define _APS_NEXT_CONTROL_VALUE		1546
#define _APS_NEXT_SYMED_VALUE		101
#define _APS_NEXT_COMMAND_VALUE		32771
#endif
#endif

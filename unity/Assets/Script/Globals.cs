using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

namespace com.rainbow.external
{
    public static class Globals
    {

        public enum ProgramMode
        {
            Real = 0,
            Simulation = 1,
            FreeDrive = 2
        }

        public static bool IsReceivedData = false;

        public static string serverIP;
        public static int serverPort;

        public static int dataLength = 0;

        public static bool isPaused = false;
        public static bool IsParsingError = false;

        public static string DebugMessage = "";

        public static bool isReceivedData = false;
        public static bool receiveCfg = false;
        public static bool saveSendFlag = false;


        public static List<string> actionAddList = new List<string>();
        public static List<string> siaMessageList = new List<string>();


        public static int CFG_DATA_SZIE;




        public static int TASK_RUN_ID = 1;
        public static int TASK_RUN_NUM;
        public static int TASK_REPEAT;
        public static float TASK_RUN_TIME;
        public static int TASK_STATE;

        public static float TASK_TIMER;

        public static int TASK_PC;

        public static int POWER_STATE;
        public static float[] TCP_TARGETS = new float[6];

        public static float DEFAULT_SPEED = 0.0f;
        public static float RECOMMEND_SPEED = -1.0f;


        public static int COLLISION_STATE;
        public static int IS_FREE_DRIVE_MODE;
        public static int INIT_STATE_INFO;
        public static int INIT_ERR;

        public static float[] TFB_ANALOG_IN = new float[2];
        public static float[] TFB_ANALOG_OUT = new float[2];
        public static int[] TFB_DIGITAL_IN = new int[2];
        public static int[] TFB_DIGITAL_OUT = new int[2];
        public static int[] DIGITAL_IN_CONFIG = new int[2];




        public static int[,] JOINT_INFO_ARRAY = new int[6, 32];
        public static float[] TEMPERATURE_MCS = new float[6];

        public static float[] JOINT_CURRENTS = new float[6];

        public static float[] JOINT_ANGLE_REFS = new float[6] { 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f };
        public static float[] JOINT_ANGLES = new float[6] { 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f };


        public static float[] JOINT_MINS = new float[6];
        public static float[] JOINT_MAXS = new float[6];

        public static float[] ANALOG_IN = new float[4];
        public static float[] ANALOG_OUT = new float[4];

        public static int[] DIGITAL_IN = new int[16];
        public static int[] DIGITAL_OUT = new int[16];

        public static float[] TOOL_ANALOG_OUT = new float[2];
        public static int[] TOOL_DIGITAL_OUT = new int[2];

        public static float[] TARGET = new float[6] { 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f };

        public static float TPB_VOLTAGE_OUT;
        public static int[] IO_FUNCTION_IN = new int[16];
        public static int[] IO_FUNCTION_OUT = new int[16];
        public static int[] IP_ADDRESS = new int[4];
        public static int[] NETMASK = new int[4];
        public static int[] GATEWAY = new int[4];

        public static ProgramMode programMode = ProgramMode.Simulation;

        public static int robotState;

        public static float[] TCP_VALUES_REFS = new float[6] { 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f };
        public static float[] TCP_VALUES = new float[6];

        public static int OP_STAT_COLLISION_OCCUR;
        public static int OP_STAT_SOS_FLAG;
        public static int OP_STAT_SELF_COLLISION;
        public static int OP_STAT_ESTOP_OCCUR;
        public static int OP_STAT_EMS_FLAG;

        public static bool showWorkSpaceFlag;
        public static bool enableCollisionFlag;
        public static bool workSpaceActiveFlag;


        public static int RS485_BAUD_RATE_INDEX;
        public static int RS485_PARITY_BIT_INDEX;
        public static int RS485_STOP_BIT_INDEX;
        public static int RS485_BAUD_RATE_INDEX_BOX;
        public static int RS485_PARITY_BIT_INDEX_BOX;
        public static int RS485_STOP_BIT_INDEX_BOX;



        //cfg
        public static float SENSITIVITY = 0.0f;
        public static float[] WORK_SPACE_LIMIT = new float[6];
        public static int WORK_SPACE_ONOFF = 0;
        public static float[] MOUNT_ROTATE = new float[3];
        public static float[] TOOL_BOX_SIZE = new float[3];
        public static float[] TOOL_BOX_CENTER_POS = new float[3];
        public static float TOOL_MASS = 0.0f;
        public static float[] TOOL_MASS_CENTER_POS = new float[3];
        public static float[] TOOL_END_EFFECTOR_POS = new float[3];
        public static int USB_DETECTED_FLAG = 0;
        public static int USB_COPY_DONE_FLAG = 0;

        public static int RS485_TOOL_BAUD = 0;
        public static int RS485_TOOL_STOPBIT = 0;
        public static int RS485_TOOL_PARITYBIT = 0;
        public static int RS485_BOX_BAUD = 0;
        public static int RS485_BOX_STOPBIT = 0;
        public static int RS485_BOX_PARITYBIT = 0;
        public static int VERSION = 0;

    }
}

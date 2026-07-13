using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

namespace com.rainbow.external
{
    class ActionAPI
    {
        static RBSocket cmdSocket;
        public static bool cmdConfirmFlag = false;
        public static bool moveCmdFlag = false;

        public const int MOVE_CMD_IGNORE_COUNT = 3;
        public static int moveCmdCnt = 0;


        public static void SetSocket(RBSocket socket)
        {
            cmdSocket = socket;
        }


        public static bool IsMotionIdle()
        {
            return ((cmdConfirmFlag == true) && (Globals.robotState == 1));
        }


        public static void CobotInit()
        {
            if (cmdSocket.IsConnected == false)
            {
                return;
            }

            string sendStr = "mc jall init";
            byte[] sendBuf = Encoding.UTF8.GetBytes(sendStr);
            cmdConfirmFlag = false;
            cmdSocket.Write(sendBuf);
        }


        public static void ProgramMode_Real()
        {
            if (cmdSocket.IsConnected == false)
            {
                return;
            }

            string sendStr = "pgmode real";
            byte[] sendBuf = Encoding.UTF8.GetBytes(sendStr);
            cmdConfirmFlag = false;
            cmdSocket.Write(sendBuf);
        }


        public static void ProgramMode_Simulation()
        {
            if (cmdSocket.IsConnected == false)
            {
                return;
            }

            string sendStr = "pgmode simulation";
            byte[] sendBuf = Encoding.UTF8.GetBytes(sendStr);
            cmdConfirmFlag = false;
            cmdSocket.Write(sendBuf);
        }


        public static void MoveJoint(float joint1, float joint2, float joint3, float joint4, float joint5, float joint6, float spd = -1, float acc = -1)
        {
            if (cmdSocket.IsConnected == false)
            {
                return;
            }

            string sendStr = "jointall " + spd.ToString() + ", "
                                        + acc.ToString() + ", "
                                        + joint1.ToString() + ", "
                                        + joint2.ToString() + ", "
                                        + joint3.ToString() + ", "
                                        + joint4.ToString() + ", "
                                        + joint5.ToString() + ", "
                                        + joint6.ToString();

            byte[] sendBuf = Encoding.UTF8.GetBytes(sendStr);
            moveCmdFlag = true;
            cmdConfirmFlag = false;
            cmdSocket.Write(sendBuf);
            Globals.robotState = 3; // Moving
        }


        public static void MoveTCP(float x, float y, float z, float rx, float ry, float rz, float spd = -1, float acc = -1)
        {
            if (cmdSocket.IsConnected == false)
            {
                return;
            }

            string sendStr = "movetcp " + spd.ToString() + ", "
                                        + acc.ToString() + ", "
                                        + x.ToString() + ", "
                                        + y.ToString() + ", "
                                        + z.ToString() + ", "
                                        + rx.ToString() + ", "
                                        + ry.ToString() + ", "
                                        + rz.ToString();

            byte[] sendBuf = Encoding.UTF8.GetBytes(sendStr);
            moveCmdFlag = true;
            cmdConfirmFlag = false;
            cmdSocket.Write(sendBuf);
            Globals.robotState = 3; // Moving
        }


        public static void MoveCircle_ThreePoint(int type, float x1, float y1, float z1, float rx1, float ry1, float rz1, float x2, float y2, float z2, float rx2, float ry2, float rz2, float spd = -1, float acc = -1)
        {
            if (cmdSocket.IsConnected == false)
            {
                return;
            }

            string typeStr;
            if (type == 0)
            {
                typeStr = "intended";
            }
            else if (type == 1)
            {
                typeStr = "constant";
            }
            else if (type == 2)
            {
                typeStr = "radial";
            }
            else
            {
                return;
            }

            string sendStr = "movecircle threepoints " + typeStr + " "
                                        + spd.ToString() + ", "
                                        + acc.ToString() + ", "
                                        + x1.ToString() + ", "
                                        + y1.ToString() + ", "
                                        + z1.ToString() + ", "
                                        + rx1.ToString() + ", "
                                        + ry1.ToString() + ", "
                                        + rz1.ToString() + ", "
                                        + x2.ToString() + ", "
                                        + y2.ToString() + ", "
                                        + z2.ToString() + ", "
                                        + rx2.ToString() + ", "
                                        + ry2.ToString() + ", "
                                        + rz2.ToString();

            byte[] sendBuf = Encoding.UTF8.GetBytes(sendStr);
            moveCmdFlag = true;
            cmdConfirmFlag = false;
            cmdSocket.Write(sendBuf);
            Globals.robotState = 3; // Moving
        }


        public static void MoveCircle_Axis(int type, float cx, float cy, float cz, float ax, float ay, float az, float rot_angle, float spd = -1, float acc = -1)
        {
            if (cmdSocket.IsConnected == false)
            {
                return;
            }

            string typeStr;
            if (type == 0)
            {
                typeStr = "intended";
            }
            else if (type == 1)
            {
                typeStr = "constant";
            }
            else if (type == 2)
            {
                typeStr = "radial";
            }
            else
            {
                return;
            }

            string sendStr = "movecircle axis " + typeStr + " "
                                        + spd.ToString() + ", "
                                        + acc.ToString() + ", "
                                        + rot_angle.ToString() + ", "
                                        + cx.ToString() + ", "
                                        + cy.ToString() + ", "
                                        + cz.ToString() + ", "
                                        + ax.ToString() + ", "
                                        + ay.ToString() + ", "
                                        + az.ToString();

            byte[] sendBuf = Encoding.UTF8.GetBytes(sendStr);
            moveCmdFlag = true;
            cmdConfirmFlag = false;
            cmdSocket.Write(sendBuf);
            Globals.robotState = 3; // Moving
        }


        public static void MoveJointBlend_Clear()
        {
            if (cmdSocket.IsConnected == false)
            {
                return;
            }

            string sendStr = "blend_jnt clear_pt";
            byte[] sendBuf = Encoding.UTF8.GetBytes(sendStr);
            cmdConfirmFlag = false;
            cmdSocket.Write(sendBuf);
        }


        public static void MoveJointBlend_AddPoint(float joint1, float joint2, float joint3, float joint4, float joint5, float joint6, float spd = -1, float acc = -1)
        {
            if (cmdSocket.IsConnected == false)
            {
                return;
            }

            string sendStr = "blend_jnt add_pt " + spd.ToString() + ", "
                                        + acc.ToString() + ", "
                                        + joint1.ToString() + ", "
                                        + joint2.ToString() + ", "
                                        + joint3.ToString() + ", "
                                        + joint4.ToString() + ", "
                                        + joint5.ToString() + ", "
                                        + joint6.ToString();

            byte[] sendBuf = Encoding.UTF8.GetBytes(sendStr);
            cmdConfirmFlag = false;
            cmdSocket.Write(sendBuf);
        }


        public static void MoveJointBlend_MovePoint()
        {
            if (cmdSocket.IsConnected == false)
            {
                return;
            }

            string sendStr = "blend_jnt move_pt";
            byte[] sendBuf = Encoding.UTF8.GetBytes(sendStr);
            moveCmdFlag = true;
            cmdConfirmFlag = false;
            cmdSocket.Write(sendBuf);
            Globals.robotState = 3; // Moving
        }


        public static void MoveTCPBlend_Clear()
        {
            if (cmdSocket.IsConnected == false)
            {
                return;
            }

            string sendStr = "blend_tcp clear_pt";
            byte[] sendBuf = Encoding.UTF8.GetBytes(sendStr);
            cmdConfirmFlag = false;
            cmdSocket.Write(sendBuf);
        }


        public static void MoveTCPBlend_AddPoint(float radius, float x, float y, float z, float rx, float ry, float rz, float spd = -1, float acc = -1)
        {
            if (cmdSocket.IsConnected == false)
            {
                return;
            }

            string sendStr = "blend_tcp add_pt " + spd.ToString() + ", "
                                        + acc.ToString() + ", "
                                        + radius.ToString() + ", "
                                        + x.ToString() + ", "
                                        + y.ToString() + ", "
                                        + z.ToString() + ", "
                                        + rx.ToString() + ", "
                                        + ry.ToString() + ", "
                                        + rz.ToString();

            byte[] sendBuf = Encoding.UTF8.GetBytes(sendStr);
            cmdConfirmFlag = false;
            cmdSocket.Write(sendBuf);
        }


        public static void MoveTCPBlend_MovePoint()
        {
            if (cmdSocket.IsConnected == false)
            {
                return;
            }

            string sendStr = "blend_tcp move_pt";
            byte[] sendBuf = Encoding.UTF8.GetBytes(sendStr);
            moveCmdFlag = true;
            cmdConfirmFlag = false;
            cmdSocket.Write(sendBuf);
            Globals.robotState = 3; // Moving
        }


        public static void ControlBoxDigitalOut(int d0, int d1, int d2, int d3, int d4, int d5, int d6, int d7, int d8, int d9, int d10, int d11, int d12, int d13, int d14, int d15)
        {
            if (cmdSocket.IsConnected == false)
            {
                return;
            }

            string sendStr = "digital_out " + d0.ToString() + ", "
                                            + d1.ToString() + ", "
                                            + d2.ToString() + ", "
                                            + d3.ToString() + ", "
                                            + d4.ToString() + ", "
                                            + d5.ToString() + ", "
                                            + d6.ToString() + ", "
                                            + d7.ToString() + ", "
                                            + d8.ToString() + ", "
                                            + d9.ToString() + ", "
                                            + d10.ToString() + ", "
                                            + d11.ToString() + ", "
                                            + d12.ToString() + ", "
                                            + d13.ToString() + ", "
                                            + d14.ToString() + ", "
                                            + d15.ToString();

            byte[] sendBuf = Encoding.UTF8.GetBytes(sendStr);
            cmdConfirmFlag = false;
            cmdSocket.Write(sendBuf);
        }


        public static void ControlBoxAnalogOut(float a0, float a1, float a2, float a3)
        {
            if (cmdSocket.IsConnected == false)
            {
                return;
            }

            string sendStr = "analog_out " + a0.ToString() + ", "
                                            + a1.ToString() + ", "
                                            + a2.ToString() + ", "
                                            + a3.ToString();

            byte[] sendBuf = Encoding.UTF8.GetBytes(sendStr);
            cmdConfirmFlag = false;
            cmdSocket.Write(sendBuf);
        }


        public static void ToolOut(int volt, int d0, int d1)
        {
            if (cmdSocket.IsConnected == false)
            {
                return;
            }

            int temp_volt = volt;
            if ((temp_volt != 12) && (temp_volt != 24))
                temp_volt = 0;

            string sendStr = "tool_out " + temp_volt.ToString() + ", "
                                        + d0.ToString() + ", "
                                        + d1.ToString();

            byte[] sendBuf = Encoding.UTF8.GetBytes(sendStr);
            cmdConfirmFlag = false;
            cmdSocket.Write(sendBuf);
        }


        public static void BaseSpeedChange(float spd)
        {
            if (cmdSocket.IsConnected == false)
            {
                return;
            }

            if (spd > 1.0)
                spd = 1.0f;
            if (spd < 0.0)
                spd = 0.0f;

            string sendStr = "sdw default_speed " + spd.ToString();
            byte[] sendBuf = Encoding.UTF8.GetBytes(sendStr);
            cmdConfirmFlag = false;
            cmdSocket.Write(sendBuf);
        }


        public static void MotionPause()
        {
            if (cmdSocket.IsConnected == false)
            {
                return;
            }
            
            string sendStr = "task pause";
            byte[] sendBuf = Encoding.UTF8.GetBytes(sendStr);
            cmdConfirmFlag = false;
            cmdSocket.Write(sendBuf);
        }


        public static void MotionHalt()
        {
            if (cmdSocket.IsConnected == false)
            {
                return;
            }

            string sendStr = "task stop";
            byte[] sendBuf = Encoding.UTF8.GetBytes(sendStr);
            cmdConfirmFlag = false;
            cmdSocket.Write(sendBuf);
        }


        public static void MotionResume()
        {
            if (cmdSocket.IsConnected == false)
            {
                return;
            }

            string sendStr = "task resume_a";
            byte[] sendBuf = Encoding.UTF8.GetBytes(sendStr);
            cmdConfirmFlag = false;
            cmdSocket.Write(sendBuf);
        }


        public static void CollisionResume()
        {
            if (cmdSocket.IsConnected == false)
            {
                return;
            }

            string sendStr = "task resume_b";
            byte[] sendBuf = Encoding.UTF8.GetBytes(sendStr);
            cmdConfirmFlag = false;
            cmdSocket.Write(sendBuf);
        }

    }
}


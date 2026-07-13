using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;
using System.Threading.Tasks;

using System.Net;
using System.Net.Sockets;
using System.Threading;

namespace com.rainbow.external
{
    class RBSocket : IDisposable
    {
        Socket _socket;
        public string _ip;
        public int _port;
        bool _connected;

        public bool IsConnected
        {
            get { return _connected; }
        }

        

        public RBSocket(string ip, int port)
        {
            _ip = ip;
            _port = port;
        }


        public bool Connect()
        {
            try
            {
                _socket = new Socket(AddressFamily.InterNetwork, SocketType.Stream, ProtocolType.Tcp);
                _socket.Connect(_ip, _port);

                _connected = true;
                OnConnected();

                // keep-alive
                int size = sizeof(UInt32);
                UInt32 on = 1;
                UInt32 keepAliveInterval = 10000;   // Send a packet once every 10 seconds.
                UInt32 retryInterval = 1000;        // If no response, resend every second.
                byte[] inArray = new byte[size * 3];
                Array.Copy(BitConverter.GetBytes(on), 0, inArray, 0, size);
                Array.Copy(BitConverter.GetBytes(keepAliveInterval), 0, inArray, size, size);
                Array.Copy(BitConverter.GetBytes(retryInterval), 0, inArray, size * 2, size);

                _socket.IOControl(IOControlCode.KeepAliveValues, inArray, null);

                return true;
            }
            catch
            {
                Close();
            }
            return false;
        }

        public void Close()
        {
            bool doLostEvent = (_connected == true);

            _connected = false;
            if (_socket != null)
            {
                try
                {
                    _socket.Close();
                }
                catch
                {

                }

                if (doLostEvent == true)
                {
                    OnConnectionLost();
                }

                _socket = null;
            }
        }

        public event Action ActionConnected;
        protected virtual void OnConnected()
        {
            if (ActionConnected == null)
            {
                return;
            }
            ActionConnected();
        }

        public event Action ActionConnectionLost;
        protected virtual void OnConnectionLost()
        {
            if (ActionConnectionLost == null)
            {
                return;
            }
            ActionConnectionLost();
        }



        public int SendTimeout
        {
            get { return (int)_socket.GetSocketOption(SocketOptionLevel.Socket, SocketOptionName.SendTimeout); }
            set { _socket.SetSocketOption(SocketOptionLevel.Socket, SocketOptionName.SendTimeout, value); }
        }

        public int ReceiveTimeout
        {
            get { return (int)_socket.GetSocketOption(SocketOptionLevel.Socket, SocketOptionName.ReceiveTimeout); }
            set { _socket.SetSocketOption(SocketOptionLevel.Socket, SocketOptionName.ReceiveTimeout, value); }
        }


        public int Write(byte[] buffer)
        {
            if(_connected == false)
            {
                return 0;
            }

            int offset = 0;
            int size = buffer.Length;

            int totalSent = 0;

            while (true)
            {
                int sent = 0;

                try
                {
                    sent = _socket.Send(buffer, 0, size, SocketFlags.None);
                }
                catch(SocketException ex)
                {
                    if(ex.SocketErrorCode == SocketError.TimedOut)
                    {
                        break;
                    }

                    Close();
                    break;
                }

                totalSent += sent;

                if(totalSent == buffer.Length)
                {
                    break;
                }

                offset += sent;
                size -= sent;
            }

            return totalSent;
        }

        


        public int Read(byte[] buffer, int size)
        {
            if(_connected == false)
            {
                return 0;
            }

            if(size == 0)
            {
                return 0;
            }
            
            
            int readLen = 0;
            try
            {
                readLen = _socket.Receive(buffer, 0, size, SocketFlags.None);
            }
            catch(SocketException ex)
            {
                if(ex.SocketErrorCode == SocketError.TimedOut)
                {
                    return -1;
                }
            }

            if(readLen == 0)
            {
                Close();
                return -1;
            }

            return readLen;
        }
         

        void IDisposable.Dispose()
        {
            Close();
        }
    }
}

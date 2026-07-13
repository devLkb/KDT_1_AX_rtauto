using System;
using System.Collections;

public class BufferUtil
{
    public static byte[] AppendTwoByteArrays(byte[] arrayA, byte[] arrayB)
    {
        byte[] outputBytes = new byte[arrayA.Length + arrayB.Length];
        Buffer.BlockCopy(arrayA, 0, outputBytes, 0, arrayA.Length);
        Buffer.BlockCopy(arrayB, 0, outputBytes, arrayA.Length, arrayB.Length);
        return outputBytes;
    }

    public static Int16 getInt16(byte[] data, int offset)
    {

        if (data == null) return 0;
        if (data.Length < offset + 2) return 0;

        try
        {
            byte[] buff = new byte[2];
            Buffer.BlockCopy(data, offset, buff, 0, 2);

            return BitConverter.ToInt16(buff, 0);

        }
        catch (Exception e)
        {
            //Debug.Log("BufferUtil:getInt error : " + e.ToString());
        }

        return 0;
    }

    public static Int32 getInt32(byte[] data, int offset)
    {

        if (data == null) return 0;
        if (data.Length < offset + 4) return 0;

        try
        {
            byte[] buff = new byte[4];
            Buffer.BlockCopy(data, offset, buff, 0, 4);

            return BitConverter.ToInt32(buff, 0);

        }
        catch (Exception e)
        {
            //Debug.Log("BufferUtil:getInt error : " + e.ToString());
        }

        return 0;
    }

    public static float getFloat(byte[] data, int offset)
    {

        if (data == null) return 0;
        if (data.Length < offset + 4) return 0;

        try
        {
            byte[] buff = new byte[4];
            Buffer.BlockCopy(data, offset, buff, 0, 4);

            float fData = BitConverter.ToSingle(buff, 0);

            if (float.IsNaN(fData)) return 0;
            return fData;

        }
        catch (Exception e)
        {
            //Debug.Log("BufferUtil:getFloat error : " + e.ToString());
        }

        return 0;
    }


    public static double getDouble(byte[] data, int offset)
    {

        if (data == null) return 0;
        if (data.Length < offset + 8) return 0;

        try
        {
            byte[] buff = new byte[8];
            Buffer.BlockCopy(data, offset, buff, 0, 8);

            return BitConverter.ToDouble(buff, 0);

        }
        catch (Exception e)
        {
            //Debug.Log("BufferUtil:getDouble error : " + e.ToString());
        }

        return 0;
    }

    public static string getString(byte[] data, int offset, int length)
    {

        if (data == null) return "";
        if (data.Length < offset + length) return "";

        try
        {
            byte[] buff = new byte[length];
            Buffer.BlockCopy(data, offset, buff, 0, length);

            return BitConverter.ToString(buff);

        }
        catch (Exception e)
        {
            //Debug.Log("BufferUtil:getString error : " + e.ToString());
        }

        return "";
    }

    public static byte[] copyBytes(byte[] data, int offset, int length)
    {

        if (data == null)
            return null;

        if (data.Length <= offset)
            return null;

        if (data.Length < offset + length)
        {
            length = data.Length - offset;
        }

        byte[] result = new byte[length];
        Buffer.BlockCopy(data, offset, result, 0, length);

        return result;
    }

    public static void memcopy(byte[] src, int srcOffset, byte[] dest, int destOffset, int length)
    {

        Buffer.BlockCopy(src, srcOffset, dest, destOffset, length);
    }

}

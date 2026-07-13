using UnityEngine;

public class JointOperation : MonoBehaviour
{
    public Transform[] joints; // 0~5번 조인트
    public Transform endEffector;
    public Transform target; // 목표 큐브

    public float threshold = 0.01f;
    public int maxIterations = 10;
    public float rotationSpeed = 5f;
    // 각 조인트의 제한 각도 범위 (in degrees)
    public float[] minLimits = { -180f, -180f, -180f, -180f, -180f, -180f };
    public float[] maxLimits = { 180f, 180f, 180f, 180f, 180f, 180f };

    private float[] initialAngles;     // ✅ 초기 회전값 저장용

    void Start()
    {
        // ✅ 시작 시 각 조인트의 초기 각도 저장
        initialAngles = new float[joints.Length];
        for (int i = 0; i < joints.Length; i++)
        {
            Vector3 euler = joints[i].localEulerAngles;
            initialAngles[i] = GetRotationAngleForAxis(euler, i);
        }
    }

    void Update()
    {
        SolveIK();
    }

    void SolveIK()
    {
        int iteration = 0;

        while (iteration < maxIterations && Vector3.Distance(endEffector.position, target.position) > threshold)
        {
            for (int i = joints.Length - 1; i >= 0; i--)
            {
                Transform joint = joints[i];
                Vector3 axis = GetRotationAxis(i); // 로컬 회전축

                Vector3 toEnd = endEffector.position - joint.position;
                Vector3 toTarget = target.position - joint.position;

                float angle = Vector3.SignedAngle(toEnd, toTarget, joint.TransformDirection(axis));

                joint.Rotate(axis, angle * rotationSpeed * Time.deltaTime, Space.Self);

                ClampJointRotation(joint, i);
            }

            iteration++;
        }
    }

    Vector3 GetRotationAxis(int index)
    {
        switch (index)
        {
            case 0: return Vector3.up;
            case 1: return Vector3.right;
            case 2: return Vector3.right;
            case 3: return Vector3.up;
            case 4: return Vector3.right;
            case 5: return Vector3.up;
            default: return Vector3.zero;
        }
    }

    void ClampJointRotation(Transform joint, int index)
    {
        Vector3 euler = joint.localEulerAngles;

        switch (index)
        {
            case 0:
            case 3:
            case 5: // Y축
                float y = NormalizeAngle(euler.y);
                y = Mathf.Clamp(y, minLimits[index], maxLimits[index]);
                joint.localEulerAngles = new Vector3(euler.x, y, euler.z);
                break;

            case 1:
            case 2:
            case 4: // X축
                float x = NormalizeAngle(euler.x);
                x = Mathf.Clamp(x, minLimits[index], maxLimits[index]);
                joint.localEulerAngles = new Vector3(x, euler.y, euler.z);
                break;
        }
    }

    float NormalizeAngle(float angle)
    {
        angle %= 360f;
        if (angle > 180f) angle -= 360f;
        return angle;
    }

    /// ✅ 조인트 축에 따른 회전값 반환 (X 또는 Y)
    float GetRotationAngleForAxis(Vector3 euler, int index)
    {
        if (index == 0 || index == 3 || index == 5) // Y축
            return NormalizeAngle(euler.y);
        else                                        // X축
            return NormalizeAngle(euler.x);
    }

    /// ✅ 조인트의 회전 변화량(도)을 반환
    public float GetJointDeltaAngle(int index)
    {
        if (index < 0 || index >= joints.Length)
            return 0f;

        Vector3 euler = joints[index].localEulerAngles;
        float currentAngle = GetRotationAngleForAxis(euler, index);
        float delta = currentAngle - initialAngles[index];

        // 변화량을 -180~+180 범위로 정규화
        return NormalizeAngle(delta);
    }
}

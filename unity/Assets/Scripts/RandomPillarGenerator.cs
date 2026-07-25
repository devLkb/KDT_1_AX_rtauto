using UnityEngine;

public static class RandomPillarGenerator
{
    public enum PillarShape { Box, Cylinder }

    public static GameObject Create(
        Vector3 basePosition,
        PillarShape shape,
        float minHeight,
        float maxHeight,
        float minWidth,
        float maxWidth,
        Transform parent = null,
        Material material = null)
    {
        PrimitiveType primitive = shape == PillarShape.Cylinder ? PrimitiveType.Cylinder : PrimitiveType.Cube;
        GameObject pillar = GameObject.CreatePrimitive(primitive);
        pillar.name = shape == PillarShape.Cylinder ? "CylinderPillar" : "BoxPillar";
        if (parent != null) pillar.transform.SetParent(parent, false);

        float height = Random.Range(minHeight, maxHeight);
        float width = Random.Range(minWidth, maxWidth);

        // Unity 기본 Cylinder는 height=2, diameter=1이라 스케일 Y는 height의 절반을 써야 한다.
        // Cube는 1x1x1이라 스케일이 곧 실제 치수다.
        pillar.transform.localScale = shape == PillarShape.Cylinder
            ? new Vector3(width, height * 0.5f, width)
            : new Vector3(width, height, width);

        // basePosition은 부모 유무와 무관하게 항상 월드 좌표의 밑면 위치를 의미한다.
        pillar.transform.position = basePosition + Vector3.up * (height * 0.5f);

        if (material != null)
            pillar.GetComponent<Renderer>().sharedMaterial = material;

        return pillar;
    }
}

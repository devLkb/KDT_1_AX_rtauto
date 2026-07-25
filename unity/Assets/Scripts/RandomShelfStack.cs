using System.Collections.Generic;
using UnityEngine;

public class RandomShelfStack : MonoBehaviour
{
    public int plateCount = 5;
    public float minWidth = 0.3f;
    public float maxWidth = 1.0f;
    public float minDepth = 0.3f;
    public float maxDepth = 1.0f;
    public float thickness = 0.02f;
    public float minGap = 0.15f;
    public float maxGap = 0.4f;
    public Material plateMaterial;

    readonly List<GameObject> _plates = new List<GameObject>();

    void Start()
    {
        Build();
    }

    [ContextMenu("Build Shelves")]
    public List<GameObject> Build()
    {
        Clear();

        float topSurface = 0f;
        for (int i = 0; i < plateCount; i++)
        {
            float width = Random.Range(minWidth, maxWidth);
            float depth = Random.Range(minDepth, maxDepth);
            float gap = Random.Range(minGap, maxGap);
            float bottomOfPlate = topSurface + gap;
            float centerY = bottomOfPlate + thickness * 0.5f;

            GameObject plate = GameObject.CreatePrimitive(PrimitiveType.Cube);
            plate.name = $"Shelf_{i:00}";
            plate.transform.SetParent(transform, false);
            plate.transform.localScale = new Vector3(width, thickness, depth);
            plate.transform.localPosition = new Vector3(0f, centerY, 0f);

            if (plateMaterial != null)
                plate.GetComponent<Renderer>().sharedMaterial = plateMaterial;

            _plates.Add(plate);
            topSurface = bottomOfPlate + thickness;
        }

        return _plates;
    }

    [ContextMenu("Clear Shelves")]
    public void Clear()
    {
        foreach (GameObject plate in _plates)
        {
            if (plate == null) continue;
            if (Application.isPlaying) Destroy(plate);
            else DestroyImmediate(plate);
        }
        _plates.Clear();
    }
}

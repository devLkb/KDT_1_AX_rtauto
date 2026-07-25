using UnityEngine;

public class RandomPillarSpawner : MonoBehaviour
{
    public RandomPillarGenerator.PillarShape shape = RandomPillarGenerator.PillarShape.Cylinder;
    public float minHeight = 0.5f;
    public float maxHeight = 2.0f;
    public float minWidth = 0.1f;
    public float maxWidth = 0.4f;
    public Material material;

    GameObject _pillar;
    public GameObject Pillar => _pillar;

    void Start()
    {
        Spawn();
    }

    [ContextMenu("Spawn Pillar")]
    public void Spawn()
    {
        Clear();
        _pillar = RandomPillarGenerator.Create(
            transform.position, shape, minHeight, maxHeight, minWidth, maxWidth,
            parent: transform, material: material);
    }

    [ContextMenu("Clear Pillar")]
    public void Clear()
    {
        if (_pillar == null) return;
        if (Application.isPlaying) Destroy(_pillar);
        else DestroyImmediate(_pillar);
        _pillar = null;
    }
}

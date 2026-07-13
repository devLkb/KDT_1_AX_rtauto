using UnityEditor;
using UnityEditor.SceneManagement;
using UnityEngine;

// DG5F 핸드 변형(오른/왼 x 일반/숏) 교체 유틸.
// Tools/DG5F 메뉴에서 선택하면 씬의 현재 DG5F 핸드를 같은 위치/부모로 교체한다.
public static class DG5FVariantSwitcher
{
    static readonly string[] VariantNames =
    {
        "dg5f_right", "dg5f_right_short", "dg5f_left", "dg5f_left_short",
    };

    [MenuItem("Tools/DG5F/Right")]
    static void UseRight() { Switch("dg5f_right"); }

    [MenuItem("Tools/DG5F/Right Short")]
    static void UseRightShort() { Switch("dg5f_right_short"); }

    [MenuItem("Tools/DG5F/Left")]
    static void UseLeft() { Switch("dg5f_left"); }

    [MenuItem("Tools/DG5F/Left Short")]
    static void UseLeftShort() { Switch("dg5f_left_short"); }

    static void Switch(string variant)
    {
        if (EditorApplication.isPlaying)
        {
            Debug.LogWarning("[DG5F] Play 모드에서는 교체할 수 없습니다. 정지 후 다시 시도하세요.");
            return;
        }

        var prefab = AssetDatabase.LoadAssetAtPath<GameObject>($"Assets/Robots/Prefabs/{variant}.prefab");
        if (prefab == null)
        {
            Debug.LogError($"[DG5F] 프리팹을 찾을 수 없습니다: Assets/Robots/Prefabs/{variant}.prefab");
            return;
        }

        // 씬에서 현재 변형 탐색 (비활성 포함 — FindObjectsByType은 씬 오브젝트만 반환)
        GameObject current = null;
        foreach (var go in Object.FindObjectsByType<GameObject>(FindObjectsInactive.Include))
        {
            if (System.Array.IndexOf(VariantNames, go.name) >= 0)
            {
                current = go;
                break;
            }
        }

        Vector3 pos = Vector3.zero;
        Quaternion rot = Quaternion.identity;
        Transform parent = null;
        if (current != null)
        {
            if (current.name == variant)
            {
                Debug.Log($"[DG5F] 이미 {variant} 사용 중입니다.");
                return;
            }
            pos = current.transform.position;
            rot = current.transform.rotation;
            parent = current.transform.parent;
            Undo.DestroyObjectImmediate(current);
        }

        var instance = (GameObject)PrefabUtility.InstantiatePrefab(prefab);
        Undo.RegisterCreatedObjectUndo(instance, "Switch DG5F Variant");
        instance.transform.SetParent(parent, false);
        instance.transform.SetPositionAndRotation(pos, rot);
        Selection.activeGameObject = instance;

        EditorSceneManager.MarkSceneDirty(instance.scene);
        Debug.Log($"[DG5F] 핸드 교체 완료 → {variant} (위치/부모 유지)");
    }
}

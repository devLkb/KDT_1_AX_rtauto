using System.Collections.Generic;
using UnityEngine;

namespace KDT.GraspTraining
{
    /// <summary>
    /// Dg5fGraspAgent가 OnEpisodeBegin 초반에 직접 Randomize()를 호출한다(공보다 먼저 배치돼야
    /// 공 스폰 로직이 장애물 윗면 좌표를 참고할 수 있음). 매 호출마다 패널 위 장애물(기둥/선반)의
    /// 위치를 다시 뽑고 각자의 Spawn()/Build()로 크기·간격도 함께 재생성하며, 각 장애물의 윗면
    /// 좌표를 TopSurfacePoints에 모아 공 스폰이 그 위에 올라앉을 수 있게 한다.
    /// </summary>
    public class PanelObstacleRandomizer : MonoBehaviour
    {
        public Dg5fGraspAgent agent;
        public List<MonoBehaviour> obstacles = new List<MonoBehaviour>();

        // 로봇 베이스 바로 옆(0.10m) ~ 패널 반폭(0.90)에 내접하는 원 안쪽까지 전 구간 허용.
        // 실제 하한은 robotClearance로 로봇 콜라이더와 겹치지 않게 별도 검사한다.
        public float minRadius = 0.10f;
        public float maxRadius = Dg5fGraspSpec.PanelWidth * 0.5f - 0.05f;
        public float minSeparation = 0.25f;

        // 로봇 콜라이더 표면에서 장애물 중심까지 최소로 유지할 거리(로봇 자체 반경 + 장애물
        // 반너비 여유분 포함). 이 이하로 겹치면 시작부터 로봇이 못 움직일 수 있어 후보에서 제외한다.
        public float robotClearance = 0.35f;

        // 매 에피소드 obstacles 중 이 범위(포함)에서 개수를 뽑아 그만큼만 활성화한다.
        public int minCount = 0;
        public int maxCount = 5;

        public List<Vector3> TopSurfacePoints { get; } = new List<Vector3>();

        const int MaxPlacementAttempts = 200;

        void Awake()
        {
            if (obstacles.Count == 0)
            {
                obstacles.AddRange(GetComponentsInChildren<RandomPillarSpawner>(true));
                obstacles.AddRange(GetComponentsInChildren<RandomShelfStack>(true));
            }
        }

        [ContextMenu("Randomize Now")]
        public void Randomize()
        {
            TopSurfacePoints.Clear();

            if (agent == null || agent.robotBase == null)
            {
                Debug.LogWarning("[PanelObstacleRandomizer] agent 또는 robotBase가 비어있어 Randomize를 건너뜁니다.");
                return;
            }
            if (obstacles.Count == 0)
            {
                Debug.LogWarning("[PanelObstacleRandomizer] obstacles 목록이 비어있습니다.");
                return;
            }

            Collider[] robotColliders = agent.robotBase.GetComponentsInChildren<Collider>(true);

            var shuffled = new List<MonoBehaviour>(obstacles);
            Shuffle(shuffled);
            int activeCount = Mathf.Clamp(Random.Range(minCount, maxCount + 1), 0, shuffled.Count);

            var placed = new List<Vector2>();
            for (int i = 0; i < shuffled.Count; i++)
            {
                MonoBehaviour obstacle = shuffled[i];
                if (obstacle == null) continue;

                bool active = i < activeCount;
                obstacle.gameObject.SetActive(active);
                if (!active) continue;

                Vector2 localXZ = PickLocalXZ(placed, robotColliders);
                placed.Add(localXZ);
                obstacle.transform.position =
                    agent.robotBase.TransformPoint(new Vector3(localXZ.x, 0f, localXZ.y));

                if (obstacle is RandomPillarSpawner pillar)
                {
                    pillar.Spawn();
                    if (pillar.Pillar != null) AddTopSurfacePoint(pillar.Pillar);
                }
                else if (obstacle is RandomShelfStack shelf)
                {
                    foreach (GameObject plate in shelf.Build())
                        AddTopSurfacePoint(plate);
                }
            }
            Debug.Log($"[PanelObstacleRandomizer] Randomize() 실행, {activeCount}/{shuffled.Count}개 활성화, "
                + $"top surface 후보 {TopSurfacePoints.Count}개.");
        }

        void AddTopSurfacePoint(GameObject obstacleGO)
        {
            Renderer renderer = obstacleGO.GetComponent<Renderer>();
            if (renderer == null) return;
            Bounds b = renderer.bounds;
            TopSurfacePoints.Add(new Vector3(b.center.x, b.max.y, b.center.z));
        }

        /// obstacles가 이번 에피소드에 놓은 장애물 중 하나의 윗면 좌표를 무작위로 하나 돌려준다.
        /// 후보가 없으면(장애물 0개 등) false.
        public bool TryGetRandomTopSurfacePoint(out Vector3 worldPoint)
        {
            if (TopSurfacePoints.Count == 0)
            {
                worldPoint = default;
                return false;
            }
            worldPoint = TopSurfacePoints[Random.Range(0, TopSurfacePoints.Count)];
            return true;
        }

        static void Shuffle(List<MonoBehaviour> list)
        {
            for (int i = list.Count - 1; i > 0; i--)
            {
                int j = Random.Range(0, i + 1);
                (list[i], list[j]) = (list[j], list[i]);
            }
        }

        // robotClearance는 절대 양보하지 않는 하드 제약이고, minSeparation(장애물 간 간격)은
        // 미관상 조건이라 로봇 안전 후보를 못 찾을 때만 양보한다.
        Vector2 PickLocalXZ(List<Vector2> placed, Collider[] robotColliders)
        {
            Vector2 bestSafeCandidate = Vector2.zero;
            bool haveSafeCandidate = false;

            for (int attempt = 0; attempt < MaxPlacementAttempts; attempt++)
            {
                float radius = Random.Range(minRadius, maxRadius);
                float azimuth = Random.Range(0f, Mathf.PI * 2f);
                var candidate = new Vector2(Mathf.Cos(azimuth) * radius, Mathf.Sin(azimuth) * radius);

                if (TooCloseToRobot(candidate, robotColliders)) continue;
                if (!haveSafeCandidate)
                {
                    bestSafeCandidate = candidate;
                    haveSafeCandidate = true;
                }

                bool farEnoughFromOthers = true;
                foreach (Vector2 existing in placed)
                {
                    if (Vector2.Distance(candidate, existing) < minSeparation)
                    {
                        farEnoughFromOthers = false;
                        break;
                    }
                }
                if (farEnoughFromOthers) return candidate;
            }

            if (haveSafeCandidate) return bestSafeCandidate;

            Debug.LogWarning("[PanelObstacleRandomizer] robotClearance를 만족하는 위치를 찾지 못했습니다 — "
                + "minRadius/maxRadius/robotClearance 설정을 확인하세요.");
            return bestSafeCandidate;
        }

        bool TooCloseToRobot(Vector2 localXZ, Collider[] robotColliders)
        {
            Vector3 worldPoint = agent.robotBase.TransformPoint(new Vector3(localXZ.x, 0f, localXZ.y));
            foreach (Collider collider in robotColliders)
            {
                if (collider == null || !collider.enabled) continue;
                Vector3 closest = collider.ClosestPoint(worldPoint);
                if (Vector3.Distance(closest, worldPoint) < robotClearance) return true;
            }
            return false;
        }
    }
}

// Dg5fLogFile.cs
// Logs/ 아래 CSV 로그 파일을 여는 **유일한** 통로. 로거마다 경로 규칙을 따로 짜다
// 덮어쓰기 사고가 반복돼 한곳으로 모았다 (2026-07-16).
//
// 왜:
//   파일명이 분 단위(yyyyMMdd_HHmm)였고 StreamWriter가 append=false(덮어쓰기)라,
//   **같은 분 안에 Play를 두 번 하면 뒤 실행이 앞 로그를 소리 없이 지웠다.** 프로브를
//   연속으로 돌리면 항상 걸리는 함정이라 "찍고 바로 딴 데로 옮긴다"는 습관이 생겼고,
//   그 습관이 결국 라이브 세션 로그를 날렸다. 파이썬 쪽도 7/6에 같은 사고로 로그를
//   통째로 잃고 "실행마다 새 파일"로 고쳤으나 분 단위까지만 고쳐 함정이 남아 있었다.
//
// 규칙: 초 단위 타임스탬프 + 이미 있으면 _2, _3… 접미사. **절대 덮지 않는다.**
//   (로그는 지워지면 복구 불가, 파일이 하나 더 생기는 건 나중에 지우면 그만)
//
// ⚠️ 새 로거를 만들 때 StreamWriter를 직접 열지 말 것 — 여기를 쓸 것.

using System;
using System.IO;
using System.Text;

public static class Dg5fLogFile
{
    /// Logs/&lt;prefix&gt;_&lt;yyyyMMdd_HHmmss&gt;.csv 를 겹치지 않게 열어 준다.
    /// <param name="prefix">파일명 접두어 (예: "thumbik", "unity_dg5f")</param>
    /// <param name="path">실제로 열린 경로 (로그 출력용)</param>
    public static StreamWriter Create(string prefix, out string path)
    {
        string dir = Path.Combine(UnityEngine.Application.dataPath, "../Logs");
        Directory.CreateDirectory(dir);
        string stamp = DateTime.Now.ToString("yyyyMMdd_HHmmss");
        int n = 1;
        while (true)
        {
            path = Path.Combine(dir, n == 1
                ? prefix + "_" + stamp + ".csv"
                : prefix + "_" + stamp + "_" + n + ".csv");
            try
            {
                // CreateNew = 이미 있으면 예외 → 덮어쓰기가 API 차원에서 불가능
                var fs = new FileStream(path, FileMode.CreateNew, FileAccess.Write, FileShare.Read);
                return new StreamWriter(fs, Encoding.UTF8);
            }
            catch (IOException) { n++; }   // 이름 충돌 — 다음 접미사로
        }
    }
}

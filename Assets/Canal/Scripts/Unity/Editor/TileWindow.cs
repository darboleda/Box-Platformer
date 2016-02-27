using UnityEngine;
using UnityEditor;

public class TileWindow : EditorWindow {

    [MenuItem("Canal/Tile Window")]
    public static void ShowWindow()
    {
        EditorWindow.GetWindow<TileWindow>();
    }

    public void OnGUI()
    {
        GUILayout.BeginArea(new Rect(0, 0, 1, 1));
        Handles.color = Color.red;
        Handles.SphereCap(0, Vector3.zero, Quaternion.identity, 20f);
        GUILayout.EndArea();
    }
}

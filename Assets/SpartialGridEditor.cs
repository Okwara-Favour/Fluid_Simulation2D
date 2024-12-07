using UnityEditor;
using UnityEngine;
using static UnityEngine.GraphicsBuffer;

[CustomEditor(typeof(SpartialGrid))]
public class SpartialGridEditor : Editor
{
    public override void OnInspectorGUI()
    {
        // Draw the default Inspector
        DrawDefaultInspector();

        // Add a custom button to the Inspector
        if (!EditorApplication.isPlaying)
        {
            SpartialGrid spartialGrid = (SpartialGrid)target;
            if (GUILayout.Button("Remake Grid"))
            {
                spartialGrid.InitializeGrid();
            }
        }
        else
        {
            EditorGUILayout.HelpBox("Grid can only be remade in Edit mode.", MessageType.Info);
        }
    }
}

using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class CustomShapes
{
    private Vector3[] vertices;
    private int[] triangles;
    private Color[] colors;

    private readonly int segments = 30;
    // Initialize the vertices, triangles, and colors arrays
    public void InitializeMesh(int count, string type)
    {
        int vertexCount;
        int triangleCount;
        switch (type.ToLower())
        {
            case "triangle":
                vertexCount = count * 3;  // Each triangle has 3 vertices
                triangleCount = count * 3; // Each triangle has 3 indices
                break;

            case "box":
                vertexCount = count * 4;  // Each box has 4 vertices
                triangleCount = count * 6; // Each box is formed by 2 triangles (6 indices)
                break;

            case "circle":
                vertexCount = count * (segments + 1); // +1 for the center of each circle
                triangleCount = count * segments * 3; // Each segment forms a triangle
                break;

            default:
                Debug.LogError("Shape type not supported.");
                return;
        }

        vertices = new Vector3[vertexCount];
        triangles = new int[triangleCount];
        colors = new Color[vertexCount];
    }

    // Function to set triangle vertices and color at index i
    public void SetTriangle(int index, Vector3 pos, float size, Color color)
    {
        // Each triangle has 3 vertices
        vertices[index * 3 + 0] = new Vector3(pos.x, pos.y + size / 2, 0);       // Top
        vertices[index * 3 + 1] = new Vector3(pos.x - size / 2, pos.y - size / 2, 0); // Bottom-left
        vertices[index * 3 + 2] = new Vector3(pos.x + size / 2, pos.y - size / 2, 0); // Bottom-right

        // Set the color for each vertex
        colors[index * 3 + 0] = color;
        colors[index * 3 + 1] = color;
        colors[index * 3 + 2] = color;

        // Set up the triangle indices
        triangles[index * 3 + 0] = index * 3;
        triangles[index * 3 + 1] = index * 3 + 1;
        triangles[index * 3 + 2] = index * 3 + 2;
    }

    // Function to set box vertices and color at index i
    public void SetBox(int index, Vector3 pos, float size, Color color)
    {
        // Each box has 4 vertices, forming 2 triangles
        vertices[index * 4 + 0] = new Vector3(pos.x - size / 2, pos.y - size / 2, 0);  // Bottom-left
        vertices[index * 4 + 1] = new Vector3(pos.x + size / 2, pos.y - size / 2, 0);  // Bottom-right
        vertices[index * 4 + 2] = new Vector3(pos.x + size / 2, pos.y + size / 2, 0);  // Top-right
        vertices[index * 4 + 3] = new Vector3(pos.x - size / 2, pos.y + size / 2, 0);  // Top-left

        // Set the color for each vertex
        colors[index * 4 + 0] = color;
        colors[index * 4 + 1] = color;
        colors[index * 4 + 2] = color;
        colors[index * 4 + 3] = color;

        // First triangle
        triangles[index * 6 + 0] = index * 4;
        triangles[index * 6 + 1] = index * 4 + 1;
        triangles[index * 6 + 2] = index * 4 + 2;

        // Second triangle
        triangles[index * 6 + 3] = index * 4;
        triangles[index * 6 + 4] = index * 4 + 2;
        triangles[index * 6 + 5] = index * 4 + 3;
    }

    // Function to set circle vertices and color at index i with a given number of segments
    public void SetCircle(int index, Vector3 pos, float radius, Color color)
    {
        // First vertex is the center of the circle
        vertices[index * (segments + 1)] = pos;
        colors[index * (segments + 1)] = color;

        float angleStep = 360f / segments;
        for (int i = 1; i <= segments; i++)
        {
            float angle = Mathf.Deg2Rad * angleStep * i;
            vertices[index * (segments + 1) + i] = new Vector3(
                pos.x + Mathf.Cos(angle) * radius,
                pos.y + Mathf.Sin(angle) * radius,
                0);
            colors[index * (segments + 1) + i] = color;
        }

        // Triangles connecting center with the edge vertices
        for (int i = 0; i < segments; i++)
        {
            triangles[index * segments * 3 + i * 3] = index * (segments + 1);
            triangles[index * segments * 3 + i * 3 + 1] = index * (segments + 1) + i + 1;
            triangles[index * segments * 3 + i * 3 + 2] = index * (segments + 1) + (i == segments - 1 ? 1 : i + 2);
        }
    }

    // Assign the vertices, triangles, and colors to the mesh
    public void ApplyMesh(ref Mesh mesh)
    {
        mesh.Clear();
        Debug.Log("Vertices count: " + vertices.Length);
        Debug.Log("Triangles count: " + triangles.Length);
        Debug.Log("First 3 vertices: " +
            vertices[0] + ", " + vertices[1] + ", " + vertices[2]);

        mesh.vertices = vertices;
        mesh.triangles = triangles;
        mesh.colors = colors;  // Assign the colors
        mesh.RecalculateBounds();
    }

    // Function to apply texture to the MeshRenderer's material
    public void ApplyTexture(MeshRenderer renderer, Texture2D texture)
    {
        renderer.material.mainTexture = texture;
    }
}

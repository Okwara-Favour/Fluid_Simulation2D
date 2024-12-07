using System.Collections;
using System.Collections.Generic;
using Unity.VisualScripting;
using System.Threading.Tasks;
using UnityEditor.Experimental.GraphView;
using UnityEditor.SceneManagement;
using UnityEngine;

public class SpartialCell
{
    public Vector3 position;
    public HashSet<uint> particleIDs = new();

    public SpartialCell(Vector3 postion)
    {
        this.position = postion;
    }
}

//[ExecuteAlways]
public class SpartialGrid : MonoBehaviour
{
    public int gridWidth = 4;         // Number of cells horizontally
    public int gridHeight = 4;        // Number of cells vertically
    public Vector2 cellSize = Vector2.one;      // Size of each cell
    public Vector3 position = Vector3.zero;
    public bool showGridInEditor = true;
    // Start is called before the first frame update
    SpartialCell[,] grid;
    Vector2Int gridSize;
    Vector2 currentCellSize;

    private readonly object cellLock = new object();
    void Start()
    {
        InitializeGrid();
    }

    public void InitializeGrid()
    {
        grid = new SpartialCell[gridWidth, gridHeight];
        gridSize = new Vector2Int(gridWidth, gridHeight);
        currentCellSize = cellSize;

        float xOrigin = position.x - (gridWidth * cellSize.x) / 2f + (cellSize.x / 2f);
        float yOrigin = position.y - (gridHeight * cellSize.y) / 2f + (cellSize.y / 2f);

        Vector3 gridOrigin = new Vector3(xOrigin, yOrigin, 0);

        for (int x = 0; x < gridWidth; x++)
        {
            for (int y = gridHeight - 1; y >= 0; y--)
            {
                Vector3 cellPosition = gridOrigin + new Vector3(x * cellSize.x, y * cellSize.y, 0);
                grid[x, y] = new SpartialCell(cellPosition);
            }
        }
    }

    public Vector2Int WorldToGridPos(Vector2 pos)
    {
        // Calculate the origin of the grid (bottom-left corner of the grid's first cell)
        float xOrigin = position.x - (gridSize.x * currentCellSize.x) / 2f + (currentCellSize.x / 2f);
        float yOrigin = position.y - (gridSize.y * currentCellSize.y) / 2f + (currentCellSize.y / 2f);

        // Calculate the offset of the world position from the grid origin
        float xOffset = pos.x - xOrigin;
        float yOffset = pos.y - yOrigin;

        // Calculate the grid coordinates by dividing the offsets by the cell size
        int gridX = Mathf.RoundToInt(xOffset / currentCellSize.x);
        int gridY = Mathf.RoundToInt(yOffset / currentCellSize.y);

        return new Vector2Int(gridX, gridY);
    }

    public void AddFluidParticleToCell(FluidParticle particle)
    {
        var gridPos = WorldToGridPos(particle.position);
        if (gridPos.x < 0 || gridPos.y < 0 || gridPos.x >= gridSize.x || gridPos.y >= gridSize.y)
        {
            return;
        }
        grid[gridPos.x, gridPos.y].particleIDs.Add(particle.id);
        particle.spartialPosition = gridPos;
    }

    public SpartialCell GetCell(int x, int y)
    {
        if (x < 0 || y < 0 || x >= gridSize.x || y >= gridSize.y)
        {
            return null;
        }
        return grid[x, y];
    }

    public HashSet<uint> GetNeighbors(Vector3 pos, float radius)
    {
        HashSet<uint> ids = new HashSet<uint>();
        var gridPos = WorldToGridPos(pos);
        if (gridPos.x < 0 || gridPos.y < 0 || gridPos.x >= gridSize.x || gridPos.y >= gridSize.y)
        {
            return ids;
        }

        int cellRange = Mathf.CeilToInt(radius / currentCellSize.x);

        int rowStart = Mathf.Max(0, gridPos.y - cellRange);
        int rowEnd = Mathf.Min(gridSize.y - 1, gridPos.y + cellRange);
        int colStart = Mathf.Max(0, gridPos.x - cellRange);
        int colEnd = Mathf.Min(gridSize.x - 1, gridPos.x + cellRange);

        // Loop through neighboring cells
        for (int row = rowStart; row <= rowEnd; row++)
        {
            for (int col = colStart; col <= colEnd; col++)
            {
                ids.AddRange(grid[col, row].particleIDs);
            }
        }

        return ids;
    }

    public void Clear()
    {
        foreach (var cell in grid)
        {
            cell.particleIDs.Clear();
        }
    }

    void OnDrawGizmos()
    {
        if (!showGridInEditor) return;

        if (grid != null)
        {
            foreach (SpartialCell cell in grid)
            {
                Gizmos.color = new Color(0.5f, 0.5f, 0.5f);
                Gizmos.DrawWireCube(cell.position, new Vector3(currentCellSize.x, currentCellSize.y, 1f));
            }
        }
    }
}

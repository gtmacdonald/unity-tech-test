// Pocket Worldz take home project.
// Greg MacDonald
// Oct 23, 2023
//
// Features:
//    * A-Star
//    * Wall avoiding.
//    * Prefers line of sight to manhattan paths.
//    * Path smoothing and interpolation.
//    * Smooth movement and rotation.
//
// Notes
//    * Uses the net standard 1.0 priority queue library here: https://github.com/BlueRaja/High-Speed-Priority-Queue-for-C-Sharp
//    * To try different dlls out go to the nuget library located here: https://www.nuget.org/packages/OptimizedPriorityQueue
//        1) Download the nuget package.
//        2) Rename nuget to zip.
//        3) Unzip and the dlls are in the lib folder.
//        4) Drag and drop into Assets/Plugin folder.
//
// Reference: https://www.redblobgames.com/pathfinding/a-star/introduction.html

using System;
using System.Collections.Generic;
using System.Linq;
using Priority_Queue;
using Unity.VisualScripting;
using UnityEngine;


public class NavGrid : MonoBehaviour
{   
    #region Static Values
    
    /// <summary>
    /// The side resolution of the square nav grid.
    /// </summary>
    private static int _sideCellRes;
    
    /// <summary>
    /// Side length of the floor plane.
    /// </summary>
    private static float _sidePlaneSize;
    
    /// <summary>
    /// A 2D boolean array. True indicates an obstruction.
    /// </summary>
    private static bool[,] _obstructionGrid;
    
    /// <summary>
    /// The bounds of the floor plane.
    /// </summary>
    private static Bounds _planeBounds;
    
    private static readonly float _planeScaleFactor = 5.0f; // TODO: Make data driven.
    
    /// <summary>
    /// Deltas for traversing around a cell coordinate.
    /// </summary>
    private static readonly int[] _deltas = { -1, 0, 1 };
    
    /// <summary>
    /// Deltas for traversing two cells deep around a cell coordinate. 
    /// </summary>
    private static readonly int[] _deltas2 = { -2, -1, 0, 1, 2};
    
    /// <summary>
    /// Signals no cell.
    /// </summary>
    private static readonly (int xCol, int yRow) NoCell = (-1, -1);
    
    /// <summary>
    /// The minimum path node count required for path smoothing to take affect.
    /// </summary>
    private static readonly int _minSmoothPathCount = 3;
    
    /// <summary>
    /// How much to scale the adjacent obstruction heuristic.
    /// </summary>
    private static float _adjWeightFactor = 1000.0f;
    
    /// <summary>
    /// Side length of a cell in game coordinates.
    /// </summary>
    private static float CellSideSize => _sidePlaneSize / _sideCellRes; 
    
    /// <summary>
    /// Half side length of a cell in game coordinates.
    /// </summary>
    private static float HalfCellSideSize => CellSideSize / 2.0f; 
    
    /// <summary>
    /// The starting corner location. This coincides with the texture and nav grid origins. 
    /// </summary>
    private static Vector3 StartingCornerPoint => new(_planeBounds.max.x, 0.0f, _planeBounds.max.z); 
    
    /// <summary>
    /// The starting center point of cell (0, 0)
    /// </summary>
    private static Vector3 StartingCenterPoint => new(StartingCornerPoint.x - HalfCellSideSize, 0.0f, StartingCornerPoint.z - HalfCellSideSize);

    #endregion

    #region Cell Helpers
    
    /// <summary>
    /// Get the cell coordinates from a position.
    /// </summary>
    /// <param name="position">A position on the plane</param>
    /// <returns>A cell coordinate.</returns>
    private static (int xCol, int yRow) CellCoordsFromPosition(Vector3 position)
    {
        var delta = (StartingCornerPoint - position) / CellSideSize;
        var xCol = (int)delta.x;
        var yRow = (int)delta.z;
        return (xCol, yRow);
    } 

    /// <summary>
    /// Calculates the cell center from statics gathered in the start event.
    /// </summary>
    /// <param name="xCol">The cell's horizontal x coordinate.</param>
    /// <param name="yRow">The cell's vertical y coordinate.</param>
    /// <returns>The center of the cell.</returns>
    private static Vector3 CellCenter(int xCol, int yRow)
    {
        var start = StartingCenterPoint;
        var newX = start.x - xCol * CellSideSize;
        var newZ = start.z - yRow * CellSideSize;
        return new Vector3(newX, 0.0f, newZ);
    }
    
    /// <summary>
    /// Calculates the cell center from statics gathered in the start event.
    /// </summary>
    /// <param name="cellCoord">The cell coordinates.</param>
    /// <returns>The center of the cell.</returns>
    private static Vector3 CellCenter((int xCol, int yRow) cellCoord)
    {
        return CellCenter(cellCoord.xCol, cellCoord.yRow);
    }
    
    /// <summary>
    /// Returns the 8 neighboring cells of a center cell.
    /// Less if the centerCell is on the edge.
    /// </summary>
    /// <param name="centerCell">The cell for which we want neighbors.</param>
    /// <returns>A list of cells.</returns>
    private static List<(int xCol, int yRow)> CellNeighbors((int xCol, int yRow) centerCell)
    {
        var neighbors = new List<(int xCol, int yRow)>();
        
        foreach (var dx in _deltas)
        {
            var x = centerCell.xCol + dx;
            if (x < 0 || x >= _sideCellRes)
            {
                continue;
            }
            foreach (var dy in _deltas)
            {
                var y = centerCell.yRow + dy;
                
                if (y < 0 || y >= _sideCellRes)
                {
                    continue;
                }

                if (_obstructionGrid[x, y])
                {
                    continue;
                }

                if (dx == 0 && dy == 0)
                {
                    continue;
                }

                neighbors.Add((x, y));
            }
        }

        return neighbors;
    }
    
    #endregion
    
    #region NavGrid
    
    /// <summary>
    /// These heuristics are the weights that help limit the number of cells visited.
    /// De-prioritizes cells that are within two cells of an obstacle.
    /// Also de-prioritizes Manhatten distances, favoring line of sight. 
    /// </summary>
    /// <param name="goal">The goal cell</param>
    /// <param name="next">The next neighbor under consideration.</param>
    /// <returns>A priority value.</returns>
    private static float Heuristic((int xCol, int yRow) goal, (int xCol, int yRow) next)
    {
        float adjWeight = 0.0f;
        foreach (var dx in _deltas2)
        {
            var x = next.xCol + dx;
            if (x < 0 || x >= _sideCellRes)
            {
                continue;
            }
            foreach (var dy in _deltas2)
            {
                var y = next.yRow + dy;
                
                if (y < 0 || y >= _sideCellRes)
                {
                    continue;
                }

                if (dx == 0 && dy == 0)
                {
                    continue;
                }
                
                if (_obstructionGrid[x, y])
                {
                    adjWeight += _adjWeightFactor * CellSideSize;
                }
            }
        }
        
        var manhattanDist = Mathf.Abs(goal.xCol - next.xCol) + Mathf.Abs(goal.yRow - next.yRow);
        
        return manhattanDist + adjWeight;
    }
    
    /// <summary>
    /// Given the current and desired location, return a path to the destination
    /// </summary>
    public NavGridPathNode[] GetPath(Vector3 startPosition, Vector3 goalPosition)
    {   
        (int, int) current;
        
        var frontier = new SimplePriorityQueue<(int xCol, int yRow)>();
        
        var start = CellCoordsFromPosition(startPosition);
        
        frontier.Enqueue(start, 0.0f);
        
        var cameFrom = new Dictionary<(int xCol, int yRow), (int xCol, int yRow)>
        {
            [start] = NoCell
        };
        
        var costSoFar = new Dictionary<(int xCol, int yRow), float>
        {
            [start] = 0.0f
        };
        
        var goal = CellCoordsFromPosition(goalPosition);

        if (_obstructionGrid[goal.xCol, goal.yRow])
        {
            return new NavGridPathNode[] { };
        }
        
        if (start == goal)
        {
            return new NavGridPathNode[]
            {
                new() { Position = startPosition },
                new() { Position = goalPosition }
            };
        }
        
        while (frontier.Count > 0)
        {
            current = frontier.Dequeue();
            
            if (current == goal)
            {
                break;
            }
        
            foreach (var next in CellNeighbors(current))
            {
                var newCost = costSoFar[current] + CostBetweenNeighbors(current, next);
                
                if (!costSoFar.ContainsKey(next) || newCost < costSoFar[next])
                {
                    costSoFar[next] = newCost;
                    var priority = newCost + Heuristic(goal, next);
                    frontier.Enqueue(next, priority);
                    cameFrom[next] = current;
                }
            }
        }
        
        // Reconstruct coord path.
        
        var coordPath = new List<(int xCol, int yRow)>();
        
        current = goal;
        
        while (current != start)
        {
            coordPath.Add(current);
            try
            {
                current = cameFrom[current];
            }
            catch 
            {
                return new NavGridPathNode[] { };
            }
        }
        
        coordPath.Add(start);
        coordPath.Reverse();
        
        // Create node path from coord path.
        
        var nodePath = new List<NavGridPathNode>();
        
        nodePath.Add(new() {Position = startPosition});
        
        // Trim ends so we preserve start and goal.
        foreach (var cell in coordPath.Skip(1).SkipLast(1))
        {
            nodePath.Add(new() { Position = CellCenter(cell)});
        }
        
        nodePath.Add(new() {Position = goalPosition});

        if (nodePath.Count < _minSmoothPathCount)
        {
            return nodePath.ToArray();
        }
        
        // Smooth path.
        
        var smoothedPath = new List<NavGridPathNode>();

        smoothedPath.Add(nodePath.First());

        for (int i = 1; i < nodePath.Count-1; ++i)
        {
            var smoothPos = nodePath[i - 1].Position * 0.25f + nodePath[i].Position * 0.5f +
                             nodePath[i + 1].Position * 0.25f;
            
            smoothedPath.Add(new() { Position = (smoothPos + nodePath[i - 1].Position)*0.5f });
            smoothedPath.Add(new() { Position = smoothPos });
            smoothedPath.Add(new() { Position = (smoothPos + nodePath[i + 1].Position)*0.5f });
        }
        
        smoothedPath.Add(nodePath.Last());
        
        return smoothedPath.ToArray();
    }

    /// <summary>
    /// Calculates the cost between neighbors in any direction. Contrast with the heuristic
    /// which is directionally focused.
    /// </summary>
    /// <param name="current">The current cell being traversed by a-star.</param>
    /// <param name="next">The next neighbor cell being considered.</param>
    /// <returns>A priority value.</returns>
    private float CostBetweenNeighbors((int xCol, int yRow) current, (int xCol, int yRow) next)
    {
        return (CellCenter(current) - CellCenter(next)).magnitude;
    }
    
    #endregion
    
    #region Events
    
    /// <summary>
    /// Gathers useful statics from the scene.
    /// Setup _obstructionGrid a 2D bool array indicating where obstructions are.
    /// </summary>
    void Start()
    {
        var material = GetComponent<Renderer>().material;
        var texture = material.mainTexture;
        _sideCellRes = texture.width;
        
        Mesh mesh = GetComponent<MeshFilter>().mesh;
        
        _planeBounds = new Bounds(mesh.bounds.center, 2.0f * mesh.bounds.extents * _planeScaleFactor);
        
        _sidePlaneSize = 2.0f * mesh.bounds.extents.x * _planeScaleFactor; 
        
        _obstructionGrid = new bool[_sideCellRes, _sideCellRes];
        
        var tex2D = texture as Texture2D;
        
        foreach (var x  in Enumerable.Range(0, _sideCellRes))
        {
            foreach (var y in Enumerable.Range(0, _sideCellRes))
            {
                // Lower left is 0, 0.
                // x == columns
                // y == rows
                // (width, height)
                if (tex2D.GetPixel(x, y) == Color.black)
                {
                    _obstructionGrid[x, y] = false;
                }
                else
                {
                    _obstructionGrid[x, y] = true;
                }
            }
        }
    }
    
    #endregion
}

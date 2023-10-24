using System;
using UnityEngine;

public class Player : MonoBehaviour
{
    private NavGridPathNode[] _currentPath = Array.Empty<NavGridPathNode>();
    private int _currentPathIndex = 0;
    
    /// <summary>
    ///  The distance when we're close enough to a node to move to the next one.
    /// </summary>
    private float _minDistance = 0.4f;

    [SerializeField]
    private NavGrid _grid;
    [SerializeField]
    private float _maxSpeed = 10.0f;
    
    
    void Update()
    {
        // Check Input
        if (Input.GetMouseButtonUp(0))
        {
            var ray = Camera.main.ScreenPointToRay(Input.mousePosition);
            if (Physics.Raycast(ray, out var hitInfo))
            {
                _currentPath = _grid.GetPath(transform.position, hitInfo.point);
                _currentPathIndex = 0;
            }
        }

        // Traverse
        if (_currentPathIndex < _currentPath.Length)
        {
            var currentNode = _currentPath[_currentPathIndex];
            
            var maxDistance = _maxSpeed * Time.deltaTime;
            var vectorToDestination = currentNode.Position - transform.position;
            var moveDistance = Mathf.Min(vectorToDestination.magnitude, maxDistance);

            var forwardDirection = vectorToDestination.normalized;
            forwardDirection.y = 0.0f;

            var rotationForward = new Quaternion();
            rotationForward.SetLookRotation(forwardDirection);
            transform.rotation = rotationForward;
            
            var moveVector = forwardDirection * moveDistance;
            transform.position += moveVector;

            if (Mathf.Abs(transform.position.x - currentNode.Position.x) <= _minDistance &&
                Mathf.Abs(transform.position.z - currentNode.Position.z) <= _minDistance)
            {
                _currentPathIndex++;
            }
        }
    }
}

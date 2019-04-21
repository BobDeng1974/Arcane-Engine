#pragma once

#define COS_30 0.866666f

#include "terrain/Terrain.h"
#include "scene/RenderableModel.h"
#include <graphics/mesh/common/Cube.h>
#include <graphics/Shader.h>
#include <graphics/renderer/GLCache.h>
#include <scene/RenderableModel.h>
#include <graphics/camera/ICamera.h>
#include <pathfinding/Obstacle.h>

namespace arcane
{
	struct TrianglePrim
	{
		union
		{
			std::array<glm::vec3*, 3> v;
			struct {
				glm::vec3* a;
				glm::vec3* b;
				glm::vec3* c;
			};
		};

		bool operator==(const TrianglePrim& other) {
			if (*(this->a) == *(other.a) && *(this->b) == *(other.b) && *(this->c) == *(other.c)) {
				return true;
			}

			return false;
		}
	};

	struct QuadPrim 
	{
		union
		{
			std::array<glm::vec3*, 4> v;
			struct {
				glm::vec3* a;
				glm::vec3* b;
				glm::vec3* c;
				glm::vec3* d;
			};
		};
	};

	class NavigationMesh
	{
	public:
		Terrain* terrain; // Reference terrain that we want to build the navigation mesh on 
		std::function<void()> regenerationCallback;

		std::unordered_map<glm::vec3*, std::unordered_set<TrianglePrim*>> m_PointToTriangle;
		
		NavigationMesh(Terrain* terrain);
		~NavigationMesh();

		static bool IsPointInTriangle(const glm::vec3& point, const TrianglePrim& triangle);
		static bool SameSide(const glm::vec3& p1, const glm::vec3& p2, const glm::vec3& a, const glm::vec3& b);
		static bool IsPointOnPlane(const glm::vec3& point, const glm::vec3& p1, const glm::vec3& p2, const glm::vec3& p3);
		static bool SameSideTriangle(const glm::vec3& point, const glm::vec3& a, const glm::vec3& b, const glm::vec3& c);
		static TrianglePrim* GetTriangleFromPoint(const glm::vec3& point, std::vector<TrianglePrim>& triangles);
		static bool TriangleLineIntersect(TrianglePrim& tri, glm::vec3& start, glm::vec3& end);
		static bool LineSegmentLineSegmentIntersection(glm::vec3& ab, glm::vec3& cd);


		// Set the terrain that this navMesh will use (Might want to regenerate mesh for new Terrain)
		inline void SetTerrain(Terrain* terrain) { this->terrain = terrain; }
		// Generate the navigation mesh from the current terrain
		void GenerateNavigationMesh();
		// Used as the callback for regeneration of the navigation mesh on the UI button click
		void OnRegenerateButtonClick();
		// Checks if there is an obstacle at this location
		bool ObstacleOnPoint(const glm::vec3& point);
		// Checks if this point is navigable
		bool ExistsPathToPoint(const glm::vec3& point, const std::vector<glm::vec3>& terrainPoints);
		// Triangulate the generated polygon
		std::vector<TrianglePrim> TriangulatePoly(std::vector<std::vector<glm::vec3*>>& polygon);
		// Draw the navigation mesh
		void DrawMesh(ICamera *camera);
		// Draw the vertices of the navigation mesh
		void DrawVertices(ICamera* camera);
		// Get the slope of the 2 pts its mostly the angle between these 2 pts and a reference vector(probably side vector)
		float GetSlopePoints(const glm::vec3& point1, const glm::vec3& point2);
		
		void AddObstacle(const Obstacle& obstacle);


		inline std::vector<TrianglePrim>& getTriangulatedPolygon() { return m_TriangulatedPolygon; }
		inline std::unordered_map<glm::vec3*, std::unordered_set<TrianglePrim*>>& getPointToTriangle() { return m_PointToTriangle; }
	private:
		void SetupVertexDebugView();
		void SetupNavmeshDebugView();
	private:
		GLCache *m_GLCache;
		ICamera *m_Camera;
		Shader* m_DebugVerticesShader, *m_DebugNavmeshShader;
		Cube m_Cube; // Mesh that we perform instancing with
		unsigned int m_CubeInstancedVAO, m_CubePositionInstancedVBO, m_CubeTransformInstancedVBO; // For vertex debug view
		unsigned int m_NavmeshVAO, m_NavmeshVBO; // For navmesh debug view
		float m_NavmeshDebugYOffset = 1.5f;

		std::vector<std::vector<glm::vec3*>> m_NavigationPolygon; 
		int m_NumCubesToDraw;

		std::vector<TrianglePrim> m_TriangulatedPolygon;
		int m_NumVerticesInNavmesh;

		// Obstacles
		std::vector<Obstacle> m_Obstacles;
	};
}

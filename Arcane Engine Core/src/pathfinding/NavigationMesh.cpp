#include "pch.h"
#include "NavigationMesh.h"

#include <ui/NavmeshPane.h>
#include <utils/loaders/ShaderLoader.h>
#include <graphics/mesh/common/Cube.h>

namespace arcane
{
	NavigationMesh::NavigationMesh(Terrain* terrain) : terrain(terrain)
	{
		SetSlopeMesh(NavmeshPane::getNavmeshSlope()); // Checks the angle given for the slope and calculates the cosine of that angle
		m_CubePositionInstancedVBO = 0;
		m_CubeTransformInstancedVBO = 0;
		m_NavmeshVBO = 0;

		regenerationCallback = [&] { OnRegenerateButtonClick(); };
		NavmeshPane::setRegenerationFunctionPtr(regenerationCallback);

		// Setup debug drawing
		m_GLCache = GLCache::getInstance();
		m_DebugVerticesShader = ShaderLoader::loadShader("src/shaders/simple_instanced.vert", "src/shaders/simple.frag");
		m_DebugNavmeshShader = ShaderLoader::loadShader("src/shaders/simple.vert", "src/shaders/simple.frag");

		OnRegenerateButtonClick();
	}

	NavigationMesh::~NavigationMesh()
	{}

	bool NavigationMesh::IsPointOnTriangle(const glm::vec3& point, const TrianglePrim& triangle)
	{
		// Check if point is on the same plane as the triangle
		if (!IsPointOnPlane(point, *triangle.a, *triangle.b, *triangle.c))
			return false;

		// Check if point is inside the triangle or outside within the plane they share
		return SameSideTriangle(point, *triangle.a, *triangle.b, *triangle.c) ||
			SameSideTriangle(point, *triangle.b, *triangle.a, *triangle.c) || 
			SameSideTriangle(point, *triangle.c, *triangle.a, *triangle.b);
	}

	bool NavigationMesh::IsPointOnPlane(const glm::vec3& point, const glm::vec3& p1, const glm::vec3& p2, const glm::vec3& p3)
	{
		glm::vec3 p1p2Crossp1p3 = glm::cross(p2 - p1, p3 - p1); // This is the normal of the plane formed by the 3 points of the plane
		float r = glm::dot(p1, p1p2Crossp1p3); // find the last unknown on the plane equation
		
		return glm::dot(point, p1p2Crossp1p3) + r == 0; // Might need to round this value a little bit
	}

	bool NavigationMesh::SameSideTriangle(const glm::vec3& point, const glm::vec3& a, const glm::vec3& b, const glm::vec3& c)
	{
		glm::vec3 abCrossap = glm::cross(b - a, point - a);
		glm::vec3 abCrossac = glm::cross(b - a, c - a);

		return glm::dot(abCrossap, abCrossac) >= 0;
	}

	void NavigationMesh::SetSlopeMesh(float angle) 
	{
		this->m_slopeAngle = std::cos(angle);
	}
	
	float NavigationMesh::GetSlopePoints(const glm::vec3& point1, const glm::vec3& point2)
	{
		glm::vec3 difference = point2 - point1; // Getting the difference vectors
		glm::vec3 referenceVec = difference;
		referenceVec.y = point1.y; // Create a vector in the direction of the difference but get straighten it out with reference point's y coordinate
		float dot = glm::dot(glm::normalize(difference), glm::normalize(referenceVec)); // This will give us cos(theta)

		return dot;
	}

	bool NavigationMesh::ObstacleOnPoint(const glm::vec3& point)
	{
		return false;
	}

	bool NavigationMesh::ExistsPathToPoint(const glm::vec3& point, const std::vector<glm::vec3>& terrainPoints)
	{
		// Check if we can go from this point to any of the others around it
		unsigned int columnNumber = terrain->GetVertexCount();
		return false;
	}

	std::vector<TrianglePrim> NavigationMesh::TriangulatePoly(std::vector<std::vector<glm::vec3*>>& polygon)
	{
		std::vector<TrianglePrim> triangles;

		// Testing purposes
		TrianglePrim prim;
		prim.a = new glm::vec3(0.0f, 100.0f, 0.0f);
		prim.b = new glm::vec3(0.0f, 100.0f, 20.0f);
		prim.c = new glm::vec3(20.0f, 100.0f, 0.0f);
		triangles.push_back(prim);

		return triangles;
	}

	void NavigationMesh::SetupVertexDebugView() {
		std::vector<glm::vec3> verticesToDraw;
		for (int i = 0; i < m_NavigationPolygon.size(); i++) {
			for (int j = 0; j < m_NavigationPolygon[i].size(); j++) {
				verticesToDraw.push_back(*(m_NavigationPolygon[i][j]));
			}
		}
		m_NumCubesToDraw = verticesToDraw.size();

		if (m_NumCubesToDraw != 0) {
			glGenVertexArrays(1, &m_CubeInstancedVAO);
			glGenBuffers(1, &m_CubePositionInstancedVBO);
			glGenBuffers(1, &m_CubeTransformInstancedVBO);

			glBindBuffer(GL_ARRAY_BUFFER, m_CubePositionInstancedVBO);
			glBufferData(GL_ARRAY_BUFFER, m_Cube.GetPositions().size() * sizeof(glm::vec3), &m_Cube.GetPositions()[0], GL_STATIC_DRAW);

			glBindBuffer(GL_ARRAY_BUFFER, m_CubeTransformInstancedVBO);
			glBufferData(GL_ARRAY_BUFFER, m_NumCubesToDraw * sizeof(glm::vec3), &verticesToDraw[0], GL_STATIC_DRAW);

			glBindVertexArray(m_CubeInstancedVAO);

			// Cube model space position
			glBindBuffer(GL_ARRAY_BUFFER, m_CubePositionInstancedVBO);
			glEnableVertexAttribArray(0);
			glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 0, NULL);
			// Transform position of each instance
			glBindBuffer(GL_ARRAY_BUFFER, m_CubeTransformInstancedVBO);
			glEnableVertexAttribArray(1);
			glVertexAttribPointer(1, 3, GL_FLOAT, GL_FALSE, 0, NULL);
			glVertexAttribDivisor(1, 1);

			glBindVertexArray(0);
		}
	}

	void NavigationMesh::SetupNavmeshDebugView() {
		std::vector<glm::vec3> triangleVertices;
		for (int i = 0; i < m_TriangulatedPolygon.size(); i++) {
			triangleVertices.push_back(*(m_TriangulatedPolygon[i].a) + glm::vec3(0.0f, m_NavmeshDebugYOffset, 0.0f));
			triangleVertices.push_back(*(m_TriangulatedPolygon[i].b) + glm::vec3(0.0f, m_NavmeshDebugYOffset, 0.0f));
			triangleVertices.push_back(*(m_TriangulatedPolygon[i].c) + glm::vec3(0.0f, m_NavmeshDebugYOffset, 0.0f));
		}
		m_NumVerticesInNavmesh = triangleVertices.size();

		if (m_NumVerticesInNavmesh != 0) {
			glGenVertexArrays(1, &m_NavmeshVAO);
			glGenBuffers(1, &m_NavmeshVBO);

			glBindVertexArray(m_NavmeshVAO);
			glBindBuffer(GL_ARRAY_BUFFER, m_NavmeshVBO);
			glBufferData(GL_ARRAY_BUFFER, triangleVertices.size() * sizeof(glm::vec3), &triangleVertices[0], GL_STATIC_DRAW);

			glEnableVertexAttribArray(0);
			glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 0, NULL);

			glBindVertexArray(0);
		}
	}

	void NavigationMesh::OnRegenerateButtonClick() {
		std::cout << "Regenerating Nav Mesh" << std::endl;

		// Clean up old instance buffers on the GPU
		if (m_CubePositionInstancedVBO != 0 && m_CubeTransformInstancedVBO != 0) {
			glDeleteVertexArrays(1, &m_CubeInstancedVAO);
			glDeleteBuffers(1, &m_CubePositionInstancedVBO);
			glDeleteBuffers(1, &m_CubeTransformInstancedVBO);
		}
		if (m_NavmeshVBO != 0) {
			glDeleteVertexArrays(1, &m_NavmeshVAO);
			glDeleteBuffers(1, &m_NavmeshVBO);
		}

		// Setup
		SetSlopeMesh(NavmeshPane::getNavmeshSlope());
		
		// Regenerate
		GenerateNavigationMesh();

		// Post generation setup
		SetupVertexDebugView();
		SetupNavmeshDebugView();
	}

	void NavigationMesh::DrawMesh(ICamera* camera) {
		m_GLCache->switchShader(m_DebugNavmeshShader);
		m_GLCache->setFaceCull(false);

		// Setup colour and also view and projection matrices
		m_DebugNavmeshShader->setUniformMat4("view", camera->getViewMatrix());
		m_DebugNavmeshShader->setUniformMat4("projection", camera->getProjectionMatrix());
		m_DebugNavmeshShader->setUniform3f("colour", glm::vec3(0.0f, 0.0f, 1.0f));

		// Draw our navmesh
		if (m_NumVerticesInNavmesh != 0) {
			glBindVertexArray(m_NavmeshVAO);
			glDrawArrays(GL_TRIANGLES, 0, m_NumVerticesInNavmesh);
		}
		m_GLCache->setFaceCull(true);
	}

	void NavigationMesh::DrawVertices(ICamera* camera) {
		m_GLCache->switchShader(m_DebugVerticesShader);

		// Setup colour and also view and projection matrices
		m_DebugVerticesShader->setUniformMat4("view", camera->getViewMatrix());
		m_DebugVerticesShader->setUniformMat4("projection", camera->getProjectionMatrix());
		m_DebugVerticesShader->setUniform3f("colour", glm::vec3(1.0f, 0.0f, 0.0f));

		// Draw our cube
		if (m_NumCubesToDraw != 0) {
			glBindVertexArray(m_CubeInstancedVAO);
			glDrawArraysInstanced(GL_TRIANGLES, 0, m_Cube.GetPositions().size(), m_NumCubesToDraw);
		}
	}

	void NavigationMesh::GenerateNavigationMesh()
	{
		// Clear the old nav mesh stuff
		m_NavigationPolygon.clear();
		m_TriangulatedPolygon.clear();

		// Filter out the points that we cannot reach
		std::vector<glm::vec3>& terrainPoints = terrain->GetPoints();
		int rowNumber = 0;
		int columnCount = terrain->GetVertexCount();
		for (int i = 0; i < terrainPoints.size(); ++i)
		{
			// Check for rows there probably is a better way to do this
			if (i == m_NavigationPolygon.size() * columnCount)
				m_NavigationPolygon.emplace_back();
	
			// Check if there is an obstacle at this point or whether it is in the list if so forget about it
			if (ObstacleOnPoint(terrainPoints[i]))
				continue; // No obstacles being checked atm

			// Check if any of the points around it can navigate to the point we are currently on 
			bool navigable = false;
			for (int j = 0; j < 2; ++j)
			{
				for (int k = -1; k < 2; ++k)
				{
					int index = (i + k) + (j * columnCount);
					if (index < 0 || index >= terrainPoints.size())
						continue;

					glm::vec3* neighborPoint = &terrainPoints[index];
					if (*neighborPoint == terrainPoints[i])
						continue;

					// Check the slope of the 2 points
					if (GetSlopePoints(terrainPoints[i], *neighborPoint) > m_slopeAngle)
					{
						m_NavigationPolygon[m_NavigationPolygon.size() - 1].push_back(&terrainPoints[i]);
						navigable = true;
						break;
					}
				}

				// If we found something 
				if (navigable)
					break;
			}
		}

		// Triangulate these new points to form a new mesh
		m_TriangulatedPolygon = TriangulatePoly(m_NavigationPolygon);

		// Optimize this mesh for pathfinding by attempting to decrease number of triangles
	}
}

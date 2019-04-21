#include "pch.h"
#include "PathfindingAgent.h"

namespace arcane
{
	PathfindingAgent::PathfindingAgent(Terrain* terrain, RenderableModel* agentsModel) : m_Terrain(terrain), m_Model(agentsModel), m_ShouldMove(false),
		m_CurrentVelocity(glm::vec3(0, 0, 0))
	{}

	PathfindingAgent::~PathfindingAgent()
	{}

	void PathfindingAgent::update(float deltaTime) {
		if (m_ShouldMove) {
			glm::vec3 dest = m_Path[m_MovementIndex];

			// Do movement
			glm::vec3 desiredVelocity = glm::normalize(dest - m_Model->getPosition()) * m_MovementSpeed;
			glm::vec3 steering = desiredVelocity - m_CurrentVelocity;
			m_CurrentVelocity += steering;
			m_Model->setPosition(m_Model->getPosition() + m_CurrentVelocity);

			// Check if it reaches the current destination, if it does update the index
			if (glm::length2(m_Model->getPosition() - dest) < 1.0f) {
				m_MovementIndex -= 1;
				if (m_MovementIndex == -1) {
					m_ShouldMove = false;
				}
			}
		}

		// Set the height of the model to the terrain's bilinear sample
		glm::vec3 pos = m_Terrain->sampleHeightfieldBilinear(m_Model->getPosition());
		m_Model->setPosition(pos);
	}

	void PathfindingAgent::resetPath(std::unordered_map<glm::vec3*, std::unordered_set<TrianglePrim*>>& pointToTri) {
		m_ShouldMove = true;
		m_Path.clear();
		PathSmoothing(pointToTri);
		m_MovementIndex = m_Path.size() - 1;
	}

	void PathfindingAgent::PathSmoothing(std::unordered_map<glm::vec3*, std::unordered_set<TrianglePrim*>>& pointToTri)
	{
		// We are starting from the destination
		glm::vec3 firstPoint = PathfindingUtil::FindCenterTriangle(*PathfindingUtil::s_Path[0]->triangle);
		m_Path.push_back(firstPoint);
		int currentIndex = 0;

		for (int i = 1; i < PathfindingUtil::s_Path.size(); ++i)
		{
			glm::vec3 center = PathfindingUtil::FindCenterTriangle(*PathfindingUtil::s_Path[currentIndex]->triangle);
			glm::vec3 nextPoint = PathfindingUtil::PathfindingUtil::FindCenterTriangle(*PathfindingUtil::s_Path[i]->triangle);
			glm::vec3 currentPath = nextPoint - center;

			// Check intersection from the start 
			bool traversable = false;
			std::deque<TrianglePrim*> searchTris { PathfindingUtil::s_Path[currentIndex]->triangle };
			std::unordered_set<TrianglePrim*> passedTris { PathfindingUtil::s_Path[currentIndex]->triangle };
			while (!searchTris.empty())
			{	
				TrianglePrim* currentTri = searchTris.front();
				if (*currentTri == *PathfindingUtil::s_Path[i]->triangle)
				{
					traversable = true;
					break;
				}
				searchTris.pop_front();
					
				for (int k = 0; k < currentTri->v.size(); ++k)
				{
					if (pointToTri.count(currentTri->v[k]) == 0)
						continue;

					std::unordered_set<TrianglePrim*> & neighborTris = pointToTri[currentTri->v[k]];
					for (auto iter = neighborTris.begin(); iter != neighborTris.end(); ++iter)
					{
						if (passedTris.count(*iter) != 0)
							continue;

						passedTris.insert(*iter);
						if (NavigationMesh::TriangleLineIntersect(*(*iter), center, nextPoint))
							searchTris.push_back(*iter);
					}
				}
			}

			// if we cannot traverse this point then add the prev one and start again
			if (!traversable)
			{
				glm::vec3 prevPoint = PathfindingUtil::PathfindingUtil::FindCenterTriangle(*PathfindingUtil::s_Path[i - 1]->triangle);
				m_Path.push_back(prevPoint); // First part of the smoothed path
				currentIndex = i - 1; // Set the new starting point
			}
		}
		m_Path.push_back(PathfindingUtil::FindCenterTriangle(*PathfindingUtil::s_Path[PathfindingUtil::s_Path.size() - 1]->triangle));
	}
}

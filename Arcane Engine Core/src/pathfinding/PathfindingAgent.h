#pragma once

#include <scene/RenderableModel.h>
#include <terrain/Terrain.h>
#include <pathfinding/NavigationMesh.h>
#include <input/InputManager.h>
#include <utils/PathfindingUtil.h>

namespace arcane
{
	class PathfindingAgent
	{
	public:
		glm::vec3 m_CurrentVelocity;

		PathfindingAgent(Terrain* terrain, RenderableModel *agentsModel);
		~PathfindingAgent();
		
		void update(float deltaTime);

		inline const glm::vec3& getPosition() { return m_Model->getPosition(); }
		
		void PathSmoothing(std::unordered_map<glm::vec3*, std::unordered_set<TrianglePrim*>>& pointToTri);
		void resetPath(std::unordered_map<glm::vec3*, std::unordered_set<TrianglePrim*>>& pointToTri);

	private:
		Terrain* m_Terrain;
		RenderableModel *m_Model;

		float m_MovementSpeed = 0.3f;
		int m_MovementIndex;
		bool m_ShouldMove;

		std::vector<glm::vec3> m_Path;
	};
}

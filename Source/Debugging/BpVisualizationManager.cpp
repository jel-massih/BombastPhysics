#include "BpVisualizationManager.h"
#include "../Foundation/BpAssert.h"

using namespace bPhysics;

void BpVisualizationManager::SetVisualizationParameter(BpVisualizationParams parameter, f32 newScale)
{
	char* message = "Attempting to set Invalid Visualization Parameter %s";
	BP_ASSERTf(parameter != BpVisualizationParams::VALUE_COUNT, message, "Test Param");

	m_visualizationParams[parameter] = newScale;
}

f32 BpVisualizationManager::GetVisualizationParameter(BpVisualizationParams parameter) const
{
	return m_visualizationParams[parameter];
}

void BpVisualizationManager::VisualizeShape(const BpShape& shape, const BpRigidActor& owner)
{
	shape.DebugVisualize(m_renderBuffer, owner);
}
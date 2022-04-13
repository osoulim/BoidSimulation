#pragma once

#include <iosfwd>
#include <string>

#include "givio.h"
#include "givr.h"
#include "imgui/imgui.h"

namespace panel {

	extern bool showPanel;
	extern ImVec4 clear_color;

// animation
	extern bool playModel;
	extern bool resetModel;
	extern bool stepModel;
	extern float dt;

	extern float separationConstant;
	extern float alignmentConstant;
	extern float cohesionConstant;

	extern int boidsNumber;
	extern int indexingMethod;

// reset
	extern bool resetView;

	void updateMenu();

} // namespace panel

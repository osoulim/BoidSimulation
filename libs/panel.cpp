#include "panel.h"

#include <array>

namespace panel {

// default values
bool showPanel = true;
ImVec4 clear_color = ImVec4(0.45f, 0.55f, 0.60f, 1.00f);

// animation
bool playModel = false;
bool resetModel = false;
bool stepModel = false;
float dt = 0.01f;

// reset
bool resetView = false;

void updateMenu() {
  using namespace ImGui;

  giv::io::ImGuiBeginFrame();

  if (showPanel && Begin("panel", &showPanel, ImGuiWindowFlags_MenuBar)) {
    if (BeginMenuBar()) {
      if (BeginMenu("File")) {
        if (MenuItem("Close", "(P)")) {
          showPanel = false;
        }
        // add more if you would like...
        ImGui::EndMenu();
      }
      EndMenuBar();
    }

    Spacing();
    if (CollapsingHeader("Background Color")) { // Clear
      ColorEdit3("Clear color", (float *)&clear_color);
    }

    Spacing();
    Separator();
    if (Button("Play/Pause")) {
      playModel = !playModel;
    }
    resetModel = Button("Reset Model");
    stepModel = Button("Step");
    InputFloat("dt", &dt, 0.00001f, 0.1f, "%.5f");

    Spacing();
    Separator();
    resetView = Button("Reset view");

    Spacing();
    Separator();
    Text("Application average %.3f ms/frame (%.1f FPS)",
         1000.0f / GetIO().Framerate, GetIO().Framerate);

    End();
  }
  giv::io::ImGuiEndFrame();
}

} // namespace panel

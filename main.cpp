/**
 * Project: Interactive ARAP
 * File:    main.cpp
 * Authors: Batuhan Erden, Cansu Yildirim, Anas Shahzad, Alexander Epple
 */

#include "include/GUI.h"

int main(int argc, char *argv[]) {
   GUI gui("resources/models/armadillo_1k.off");
   gui.launchViewer();

   return 0;
}

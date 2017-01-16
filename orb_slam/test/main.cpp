#include "parameter.h"
#include "frame.h"

#include <iostream>
#include <opencv2/highgui/highgui.hpp>

#define DEBUG_DRAW
#ifdef DEBUG_DRAW
#include "draw_image.h"
#endif // DEBUG_DRAW

int main()
{
	Parameter parameter;

#ifdef DEBUG_DRAW
	DrawImage draw_image("RGB Image", "Depth Image", true);
#endif // DEBUG_DRAW

	for (int32_t i = 1; i <= 100; i++)
	{
		std::cout << "Load image: " << i << ".png" << std::endl;
		Frame frame(i, parameter);

#ifdef DEBUG_DRAW
		draw_image.toDrawFrame(frame, 1);
#endif // DEBUG_DRAW
	}

    return 0;
}

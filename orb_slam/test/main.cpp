#include "../inc/parameter.h"
#include "../inc/frame.h"
#include "../inc/map.h"
#include "../inc/tracking.h"
#include "../inc/local_mapping.h"
#include "../inc/loop_closing.h"

#include <iostream>
#include <opencv2/highgui/highgui.hpp>

#define DEBUG_DRAW
#ifdef DEBUG_DRAW
#include "../inc/draw_image.h"
#endif // DEBUG_DRAW

int main()
{
	Parameter parameter;
	Map map;
	// LocalMapping local_mapping;
	LoopClosing loop_closing(parameter);
	Tracking tracking(parameter, &loop_closing);
	loop_closing.GetTracking(&tracking);

#ifdef DEBUG_DRAW
	DrawImage draw_image("RGB Image", "Depth Image", false);
#endif // DEBUG_DRAW

	int32_t image_number = parameter.image_number_;

	for (int32_t i = 1; i <= image_number; i++)
	{
		std::cout << "============ Load image: " << i << ".png ============" << std::endl;
		Frame *frame = new Frame(i, parameter);

#ifdef DEBUG_DRAW
		draw_image.toDrawFrame(*frame, 1);
#endif // DEBUG_DRAW

		frame->ReleaseImage();

		tracking.GetFrame(frame);

		delete frame;
	}

    return 0;
}

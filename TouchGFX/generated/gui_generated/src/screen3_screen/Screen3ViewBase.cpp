/*********************************************************************************/
/********** THIS FILE IS GENERATED BY TOUCHGFX DESIGNER, DO NOT MODIFY ***********/
/*********************************************************************************/
#include <gui_generated/screen3_screen/Screen3ViewBase.hpp>
#include <touchgfx/Color.hpp>
#include "BitmapDatabase.hpp"

Screen3ViewBase::Screen3ViewBase() :
    flexButtonCallback(this, &Screen3ViewBase::flexButtonCallbackHandler)
{

    touchgfx::CanvasWidgetRenderer::setupBuffer(canvasBuffer, CANVAS_BUFFER_SIZE);

    __background.setPosition(0, 0, 480, 272);
    __background.setColor(touchgfx::Color::getColorFrom24BitRGB(0, 0, 0));

    background.setXY(0, 0);
    background.setBitmap(touchgfx::Bitmap(BITMAP_BACKGROUND_ID));

    nextButton.setBitmaps(Bitmap(BITMAP_NEXT_BUTTON_RELEASED_ID), Bitmap(BITMAP_NEXT_BUTTON_PRESSED_ID));
    nextButton.setBitmapXY(0, 0);
    nextButton.setPosition(419, 211, 46, 46);
    nextButton.setAction(flexButtonCallback);

    gauge1.setBackground(touchgfx::Bitmap(BITMAP_GAUGE03_BACKGROUND_ID));
    gauge1.setBackgroundOffset(35, 39);
    gauge1.setPosition(80, 30, 320, 180);
    gauge1.setCenter(158, 172);
    gauge1.setStartEndAngle(-68, 68);
    gauge1.setRange(0, 100);
    gauge1.setValue(0);
    gauge1.setNeedle(BITMAP_GAUGE03_NEEDLE_ID, 6, 169);
    gauge1.setMovingNeedleRenderingAlgorithm(touchgfx::TextureMapper::BILINEAR_INTERPOLATION);
    gauge1.setSteadyNeedleRenderingAlgorithm(touchgfx::TextureMapper::BILINEAR_INTERPOLATION);
    gauge1.setArcVisible();
    gauge1Painter.setBitmap(touchgfx::Bitmap(BITMAP_GAUGE03_FILL_ID));
    gauge1.getArc().setPainter(gauge1Painter);
    gauge1.getArc().setRadius(141);
    gauge1.getArc().setLineWidth(0);
    gauge1.setArcPosition(35, 39, 250, 111);

    add(__background);
    add(background);
    add(nextButton);
    add(gauge1);
}

void Screen3ViewBase::setupScreen()
{

}

void Screen3ViewBase::flexButtonCallbackHandler(const touchgfx::AbstractButtonContainer& src)
{
    if (&src == &nextButton)
    {
        //ChangeScreen
        //When nextButton clicked change screen to Screen1
        //Go to Screen1 with no screen transition
        application().gotoScreen1ScreenNoTransition();
    }
}

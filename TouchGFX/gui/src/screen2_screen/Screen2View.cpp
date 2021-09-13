#include <gui/screen2_screen/Screen2View.hpp>
#include <BitmapDatabase.hpp>

Screen2View::Screen2View()
{

}

void Screen2View::setupScreen()
{
    Screen2ViewBase::setupScreen();
    updateIconAndText();
}

void Screen2View::tearDownScreen()
{
    Screen2ViewBase::tearDownScreen();
}

void Screen2View::handleTickEvent()
{
    // Make the gauge move from min to max value
    // Update the gauge in steps initiating an animation
    // by using updateValue(...) with a duration != 0
    static int incr = 1;
    static int timeoutCounter = 0;

    // Only update at a certain time interval (measured in ticks)
    if (timeoutCounter++ == 46)
    {
        timeoutCounter = 0;

        if (gauge1.getValue() <= 0)
        {
            incr = +17;
        }
        else if (gauge1.getValue() >= 100)
        {
            incr = -13;
        }

        // Use updateValue instead of setValue to have
        // the needle animate to the new value instead
        // of instantly move to the position.
        gauge1.updateValue(gauge1.getValue() + incr, 20);
    }
}

void Screen2View::updateIconAndText()
{
    int gaugeValue = gauge1.getValue();
    if (gaugeValue < 15)
    {
        icon.setBitmap(Bitmap(BITMAP_GAUGE02_ICON_LEVEL00_ID));
    }
    else if (gaugeValue < 50)
    {
        icon.setBitmap(Bitmap(BITMAP_GAUGE02_ICON_LEVEL01_ID));
    }
    else if (gaugeValue < 67)
    {
        icon.setBitmap(Bitmap(BITMAP_GAUGE02_ICON_LEVEL02_ID));
    }
    else if (gaugeValue < 83)
    {
        icon.setBitmap(Bitmap(BITMAP_GAUGE02_ICON_LEVEL03_ID));
    }
    else
    {
        icon.setBitmap(Bitmap(BITMAP_GAUGE02_ICON_LEVEL04_ID));
    }
    icon.invalidate();

    Unicode::snprintf(readoutTextBuffer, READOUTTEXT_SIZE, "%i", gaugeValue);
    readoutText.invalidate();
}

void Screen2View::newGaugeValue()
{
    // The icon and text should be updated each time the gauge/needle has
    // a new value during the animation
    updateIconAndText();
}

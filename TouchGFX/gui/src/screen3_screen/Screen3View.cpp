#include <gui/screen3_screen/Screen3View.hpp>

Screen3View::Screen3View()
{

}

void Screen3View::setupScreen()
{
    Screen3ViewBase::setupScreen();
}

void Screen3View::tearDownScreen()
{
    Screen3ViewBase::tearDownScreen();
}

void Screen3View::handleTickEvent()
{
    // Make the gauge move from min to max value
    static int incr = 1;

    if (gauge1.getValue() <= 0)
    {
        incr = +1;
    }
    else if (gauge1.getValue() >= 100)
    {
        incr = -1;
    }

    gauge1.setValue(gauge1.getValue() + incr);
}

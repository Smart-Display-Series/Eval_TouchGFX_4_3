#include <gui/screen1_screen/Screen1View.hpp>

Screen1View::Screen1View()
{

}

void Screen1View::setupScreen()
{
    Screen1ViewBase::setupScreen();
}

void Screen1View::tearDownScreen()
{
    Screen1ViewBase::tearDownScreen();
}

void Screen1View::handleTickEvent()
{
    // Make the gauge move from min to max value
    static int incr = 1;

    if (gauge1.getValue() <= 10)
    {
        incr = +1;
    }
    else if (gauge1.getValue() >= 140)
    {
        incr = -1;
    }

    gauge1.setValue(gauge1.getValue() + incr);
}

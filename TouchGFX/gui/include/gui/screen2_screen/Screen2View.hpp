#ifndef SCREEN2VIEW_HPP
#define SCREEN2VIEW_HPP

#include <gui_generated/screen2_screen/Screen2ViewBase.hpp>
#include <gui/screen2_screen/Screen2Presenter.hpp>

class Screen2View : public Screen2ViewBase
{
public:
    Screen2View();
    virtual ~Screen2View() {}
    virtual void setupScreen();
    virtual void tearDownScreen();

    // Callback from gauge.
    // Called each time the gauge has a new value.
    // For example in each step of the update animation
    virtual void newGaugeValue();

    void handleTickEvent();

protected:
    virtual void updateIconAndText();

};

#endif // SCREEN2VIEW_HPP

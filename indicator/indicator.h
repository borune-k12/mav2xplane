#ifndef INDICATOR_H
#define INDICATOR_H
#include <QWidget>
#include <QColor>
#include <QString>

class Indicator : public QWidget
{
    Q_OBJECT
public:
    Indicator(const QString &str, QWidget *parent = 0);
    ~Indicator(){}   
    void setColor(const QColor &color);

    void setActiveIndicator(bool isActive);

    void setText(const QString &text){m_text = text; update();}

private:
    QColor m_color;
    QString m_text;
protected:
    void paintEvent(QPaintEvent *event);
};

#endif // INDICATOR_H


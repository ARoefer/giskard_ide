#pragma once

#include <QWidget>
#include <QLineEdit>
#include <QSize>

namespace giskard_sim {
	class LineEdit : public QLineEdit {
		Q_OBJECT
	public:
		LineEdit(QWidget * parent = 0) : QLineEdit(parent) {}
		LineEdit(const QString & contents, QWidget * parent = 0) : QLineEdit(contents, parent) {}

                QSize sizeHint() const {
			return QWidget::sizeHint();
		}
        };
}

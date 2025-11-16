/********************************************************************************
** Form generated from reading UI file 'logdialog.ui'
**
** Created by: Qt User Interface Compiler version 5.9.5
**
** WARNING! All changes made in this file will be lost when recompiling UI file!
********************************************************************************/

#ifndef UI_LOGDIALOG_H
#define UI_LOGDIALOG_H

#include <QtCore/QVariant>
#include <QtWidgets/QAction>
#include <QtWidgets/QApplication>
#include <QtWidgets/QButtonGroup>
#include <QtWidgets/QDialog>
#include <QtWidgets/QDialogButtonBox>
#include <QtWidgets/QGridLayout>
#include <QtWidgets/QHeaderView>
#include <QtWidgets/QPlainTextEdit>

QT_BEGIN_NAMESPACE

class Ui_LogDialog
{
public:
    QGridLayout *gridLayout;
    QPlainTextEdit *logTextEdit;
    QDialogButtonBox *buttonBox;

    void setupUi(QDialog *LogDialog)
    {
        if (LogDialog->objectName().isEmpty())
            LogDialog->setObjectName(QStringLiteral("LogDialog"));
        LogDialog->resize(400, 300);
        gridLayout = new QGridLayout(LogDialog);
        gridLayout->setObjectName(QStringLiteral("gridLayout"));
        logTextEdit = new QPlainTextEdit(LogDialog);
        logTextEdit->setObjectName(QStringLiteral("logTextEdit"));
        logTextEdit->setReadOnly(true);

        gridLayout->addWidget(logTextEdit, 0, 0, 1, 1);

        buttonBox = new QDialogButtonBox(LogDialog);
        buttonBox->setObjectName(QStringLiteral("buttonBox"));
        buttonBox->setOrientation(Qt::Horizontal);
        buttonBox->setStandardButtons(QDialogButtonBox::Ok);
        buttonBox->setCenterButtons(false);

        gridLayout->addWidget(buttonBox, 1, 0, 1, 1);


        retranslateUi(LogDialog);
        QObject::connect(buttonBox, SIGNAL(accepted()), LogDialog, SLOT(accept()));
        QObject::connect(buttonBox, SIGNAL(rejected()), LogDialog, SLOT(reject()));

        QMetaObject::connectSlotsByName(LogDialog);
    } // setupUi

    void retranslateUi(QDialog *LogDialog)
    {
        LogDialog->setWindowTitle(QApplication::translate("LogDialog", "JTAG Diagnostic", Q_NULLPTR));
    } // retranslateUi

};

namespace Ui {
    class LogDialog: public Ui_LogDialog {};
} // namespace Ui

QT_END_NAMESPACE

#endif // UI_LOGDIALOG_H

/********************************************************************************
** Form generated from reading UI file 'livedialog.ui'
**
** Created by: Qt User Interface Compiler version 5.9.5
**
** WARNING! All changes made in this file will be lost when recompiling UI file!
********************************************************************************/

#ifndef UI_LIVEDIALOG_H
#define UI_LIVEDIALOG_H

#include <QtCore/QVariant>
#include <QtWidgets/QAction>
#include <QtWidgets/QApplication>
#include <QtWidgets/QButtonGroup>
#include <QtWidgets/QDialog>
#include <QtWidgets/QDialogButtonBox>
#include <QtWidgets/QGridLayout>
#include <QtWidgets/QGroupBox>
#include <QtWidgets/QHeaderView>

QT_BEGIN_NAMESPACE

class Ui_LiveDialog
{
public:
    QGridLayout *gridLayout;
    QDialogButtonBox *buttonBox;
    QGroupBox *sensorGroup;
    QGridLayout *gridLayout_3;
    QGridLayout *sensorLayout;
    QGroupBox *coreGroup;
    QGridLayout *gridLayout_6;
    QGridLayout *coreLayout;

    void setupUi(QDialog *LiveDialog)
    {
        if (LiveDialog->objectName().isEmpty())
            LiveDialog->setObjectName(QStringLiteral("LiveDialog"));
        LiveDialog->resize(400, 300);
        gridLayout = new QGridLayout(LiveDialog);
        gridLayout->setObjectName(QStringLiteral("gridLayout"));
        buttonBox = new QDialogButtonBox(LiveDialog);
        buttonBox->setObjectName(QStringLiteral("buttonBox"));
        buttonBox->setOrientation(Qt::Horizontal);
        buttonBox->setStandardButtons(QDialogButtonBox::Ok);

        gridLayout->addWidget(buttonBox, 2, 0, 1, 1);

        sensorGroup = new QGroupBox(LiveDialog);
        sensorGroup->setObjectName(QStringLiteral("sensorGroup"));
        gridLayout_3 = new QGridLayout(sensorGroup);
        gridLayout_3->setObjectName(QStringLiteral("gridLayout_3"));
        sensorLayout = new QGridLayout();
        sensorLayout->setObjectName(QStringLiteral("sensorLayout"));

        gridLayout_3->addLayout(sensorLayout, 0, 0, 1, 1);


        gridLayout->addWidget(sensorGroup, 0, 0, 1, 1);

        coreGroup = new QGroupBox(LiveDialog);
        coreGroup->setObjectName(QStringLiteral("coreGroup"));
        coreGroup->setEnabled(true);
        gridLayout_6 = new QGridLayout(coreGroup);
        gridLayout_6->setObjectName(QStringLiteral("gridLayout_6"));
        coreLayout = new QGridLayout();
        coreLayout->setObjectName(QStringLiteral("coreLayout"));

        gridLayout_6->addLayout(coreLayout, 0, 0, 1, 1);


        gridLayout->addWidget(coreGroup, 1, 0, 1, 1);


        retranslateUi(LiveDialog);
        QObject::connect(buttonBox, SIGNAL(accepted()), LiveDialog, SLOT(accept()));
        QObject::connect(buttonBox, SIGNAL(rejected()), LiveDialog, SLOT(reject()));

        QMetaObject::connectSlotsByName(LiveDialog);
    } // setupUi

    void retranslateUi(QDialog *LiveDialog)
    {
        LiveDialog->setWindowTitle(QApplication::translate("LiveDialog", "Dialog", Q_NULLPTR));
        sensorGroup->setTitle(QApplication::translate("LiveDialog", "Sensors", Q_NULLPTR));
        coreGroup->setTitle(QApplication::translate("LiveDialog", "Cores", Q_NULLPTR));
    } // retranslateUi

};

namespace Ui {
    class LiveDialog: public Ui_LiveDialog {};
} // namespace Ui

QT_END_NAMESPACE

#endif // UI_LIVEDIALOG_H

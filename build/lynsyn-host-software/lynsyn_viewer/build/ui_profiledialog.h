/********************************************************************************
** Form generated from reading UI file 'profiledialog.ui'
**
** Created by: Qt User Interface Compiler version 5.9.5
**
** WARNING! All changes made in this file will be lost when recompiling UI file!
********************************************************************************/

#ifndef UI_PROFILEDIALOG_H
#define UI_PROFILEDIALOG_H

#include <QtCore/QVariant>
#include <QtWidgets/QAction>
#include <QtWidgets/QApplication>
#include <QtWidgets/QButtonGroup>
#include <QtWidgets/QCheckBox>
#include <QtWidgets/QDialog>
#include <QtWidgets/QDialogButtonBox>
#include <QtWidgets/QDoubleSpinBox>
#include <QtWidgets/QGridLayout>
#include <QtWidgets/QGroupBox>
#include <QtWidgets/QHBoxLayout>
#include <QtWidgets/QHeaderView>
#include <QtWidgets/QLabel>
#include <QtWidgets/QLineEdit>
#include <QtWidgets/QVBoxLayout>

QT_BEGIN_NAMESPACE

class Ui_ProfileDialog
{
public:
    QGridLayout *gridLayout;
    QGroupBox *generalGroupBox;
    QVBoxLayout *verticalLayout_2;
    QHBoxLayout *horizontalLayout;
    QLabel *label;
    QDoubleSpinBox *periodSpinBox;
    QDialogButtonBox *buttonBox;
    QGroupBox *jtagGroupBox;
    QVBoxLayout *verticalLayout_3;
    QCheckBox *bpCheckBox;
    QHBoxLayout *horizontalLayout_2;
    QLabel *label_2;
    QLineEdit *startEdit;
    QHBoxLayout *horizontalLayout_3;
    QLabel *label_3;
    QLineEdit *stopEdit;
    QHBoxLayout *horizontalLayout_6;
    QLabel *label_4;
    QLineEdit *markEdit;
    QHBoxLayout *horizontalLayout_4;
    QLabel *label_6;
    QLineEdit *elfEdit;
    QHBoxLayout *horizontalLayout_5;
    QLabel *label_7;
    QLineEdit *kallsymsEdit;
    QLabel *coreLabel;
    QHBoxLayout *coreLayout;
    QGroupBox *sensorGroupBox;
    QVBoxLayout *sensorGroupBoxLayout;

    void setupUi(QDialog *ProfileDialog)
    {
        if (ProfileDialog->objectName().isEmpty())
            ProfileDialog->setObjectName(QStringLiteral("ProfileDialog"));
        ProfileDialog->resize(400, 412);
        gridLayout = new QGridLayout(ProfileDialog);
        gridLayout->setObjectName(QStringLiteral("gridLayout"));
        generalGroupBox = new QGroupBox(ProfileDialog);
        generalGroupBox->setObjectName(QStringLiteral("generalGroupBox"));
        verticalLayout_2 = new QVBoxLayout(generalGroupBox);
        verticalLayout_2->setObjectName(QStringLiteral("verticalLayout_2"));
        horizontalLayout = new QHBoxLayout();
        horizontalLayout->setObjectName(QStringLiteral("horizontalLayout"));
        label = new QLabel(generalGroupBox);
        label->setObjectName(QStringLiteral("label"));

        horizontalLayout->addWidget(label);

        periodSpinBox = new QDoubleSpinBox(generalGroupBox);
        periodSpinBox->setObjectName(QStringLiteral("periodSpinBox"));
        periodSpinBox->setMaximum(1e+12);

        horizontalLayout->addWidget(periodSpinBox);


        verticalLayout_2->addLayout(horizontalLayout);


        gridLayout->addWidget(generalGroupBox, 0, 0, 1, 1);

        buttonBox = new QDialogButtonBox(ProfileDialog);
        buttonBox->setObjectName(QStringLiteral("buttonBox"));
        buttonBox->setOrientation(Qt::Horizontal);
        buttonBox->setStandardButtons(QDialogButtonBox::Cancel|QDialogButtonBox::Ok);

        gridLayout->addWidget(buttonBox, 3, 0, 1, 1);

        jtagGroupBox = new QGroupBox(ProfileDialog);
        jtagGroupBox->setObjectName(QStringLiteral("jtagGroupBox"));
        verticalLayout_3 = new QVBoxLayout(jtagGroupBox);
        verticalLayout_3->setObjectName(QStringLiteral("verticalLayout_3"));
        bpCheckBox = new QCheckBox(jtagGroupBox);
        bpCheckBox->setObjectName(QStringLiteral("bpCheckBox"));

        verticalLayout_3->addWidget(bpCheckBox);

        horizontalLayout_2 = new QHBoxLayout();
        horizontalLayout_2->setObjectName(QStringLiteral("horizontalLayout_2"));
        label_2 = new QLabel(jtagGroupBox);
        label_2->setObjectName(QStringLiteral("label_2"));

        horizontalLayout_2->addWidget(label_2);

        startEdit = new QLineEdit(jtagGroupBox);
        startEdit->setObjectName(QStringLiteral("startEdit"));

        horizontalLayout_2->addWidget(startEdit);


        verticalLayout_3->addLayout(horizontalLayout_2);

        horizontalLayout_3 = new QHBoxLayout();
        horizontalLayout_3->setObjectName(QStringLiteral("horizontalLayout_3"));
        label_3 = new QLabel(jtagGroupBox);
        label_3->setObjectName(QStringLiteral("label_3"));

        horizontalLayout_3->addWidget(label_3);

        stopEdit = new QLineEdit(jtagGroupBox);
        stopEdit->setObjectName(QStringLiteral("stopEdit"));

        horizontalLayout_3->addWidget(stopEdit);


        verticalLayout_3->addLayout(horizontalLayout_3);

        horizontalLayout_6 = new QHBoxLayout();
        horizontalLayout_6->setObjectName(QStringLiteral("horizontalLayout_6"));
        label_4 = new QLabel(jtagGroupBox);
        label_4->setObjectName(QStringLiteral("label_4"));

        horizontalLayout_6->addWidget(label_4);

        markEdit = new QLineEdit(jtagGroupBox);
        markEdit->setObjectName(QStringLiteral("markEdit"));

        horizontalLayout_6->addWidget(markEdit);


        verticalLayout_3->addLayout(horizontalLayout_6);

        horizontalLayout_4 = new QHBoxLayout();
        horizontalLayout_4->setObjectName(QStringLiteral("horizontalLayout_4"));
        label_6 = new QLabel(jtagGroupBox);
        label_6->setObjectName(QStringLiteral("label_6"));

        horizontalLayout_4->addWidget(label_6);

        elfEdit = new QLineEdit(jtagGroupBox);
        elfEdit->setObjectName(QStringLiteral("elfEdit"));

        horizontalLayout_4->addWidget(elfEdit);


        verticalLayout_3->addLayout(horizontalLayout_4);

        horizontalLayout_5 = new QHBoxLayout();
        horizontalLayout_5->setObjectName(QStringLiteral("horizontalLayout_5"));
        label_7 = new QLabel(jtagGroupBox);
        label_7->setObjectName(QStringLiteral("label_7"));

        horizontalLayout_5->addWidget(label_7);

        kallsymsEdit = new QLineEdit(jtagGroupBox);
        kallsymsEdit->setObjectName(QStringLiteral("kallsymsEdit"));

        horizontalLayout_5->addWidget(kallsymsEdit);


        verticalLayout_3->addLayout(horizontalLayout_5);

        coreLabel = new QLabel(jtagGroupBox);
        coreLabel->setObjectName(QStringLiteral("coreLabel"));

        verticalLayout_3->addWidget(coreLabel);

        coreLayout = new QHBoxLayout();
        coreLayout->setObjectName(QStringLiteral("coreLayout"));

        verticalLayout_3->addLayout(coreLayout);


        gridLayout->addWidget(jtagGroupBox, 1, 0, 1, 1);

        sensorGroupBox = new QGroupBox(ProfileDialog);
        sensorGroupBox->setObjectName(QStringLiteral("sensorGroupBox"));
        sensorGroupBoxLayout = new QVBoxLayout(sensorGroupBox);
        sensorGroupBoxLayout->setObjectName(QStringLiteral("sensorGroupBoxLayout"));

        gridLayout->addWidget(sensorGroupBox, 2, 0, 1, 1);


        retranslateUi(ProfileDialog);
        QObject::connect(buttonBox, SIGNAL(accepted()), ProfileDialog, SLOT(accept()));
        QObject::connect(buttonBox, SIGNAL(rejected()), ProfileDialog, SLOT(reject()));

        QMetaObject::connectSlotsByName(ProfileDialog);
    } // setupUi

    void retranslateUi(QDialog *ProfileDialog)
    {
        ProfileDialog->setWindowTitle(QApplication::translate("ProfileDialog", "Dialog", Q_NULLPTR));
        generalGroupBox->setTitle(QApplication::translate("ProfileDialog", "General", Q_NULLPTR));
        label->setText(QApplication::translate("ProfileDialog", "Duration", Q_NULLPTR));
        jtagGroupBox->setTitle(QApplication::translate("ProfileDialog", "JTAG", Q_NULLPTR));
        bpCheckBox->setText(QApplication::translate("ProfileDialog", "Use breakpoints", Q_NULLPTR));
        label_2->setText(QApplication::translate("ProfileDialog", "Start breakpoint", Q_NULLPTR));
        label_3->setText(QApplication::translate("ProfileDialog", "Stop breakpoint", Q_NULLPTR));
        label_4->setText(QApplication::translate("ProfileDialog", "Mark breakpoint", Q_NULLPTR));
        label_6->setText(QApplication::translate("ProfileDialog", "Elf files (comma separated)", Q_NULLPTR));
        label_7->setText(QApplication::translate("ProfileDialog", "kallsyms file", Q_NULLPTR));
        coreLabel->setText(QApplication::translate("ProfileDialog", "Cores:", Q_NULLPTR));
        sensorGroupBox->setTitle(QApplication::translate("ProfileDialog", "Sensors", Q_NULLPTR));
    } // retranslateUi

};

namespace Ui {
    class ProfileDialog: public Ui_ProfileDialog {};
} // namespace Ui

QT_END_NAMESPACE

#endif // UI_PROFILEDIALOG_H

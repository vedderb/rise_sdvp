HEADERS += \
    $$PWD/include/matrix/Matrix.h \
    $$PWD/include/matrix/SparseMatrix.h \
    $$PWD/include/matrix/DenseMatrix.h \
    $$PWD/include/vector/Vector.h \
    $$PWD/include/vector/SparseVector.h \
    $$PWD/include/vector/DenseVector.h \
    $$PWD/include/decomposition/EVD.h \
    $$PWD/include/decomposition/LU.h \
    $$PWD/include/decomposition/QR.h \
    $$PWD/include/decomposition/SVD.h \
    $$PWD/include/utils/Utility.h \
    $$PWD/include/utils/Printer.h \
    $$PWD/include/utils/ArrayOperator.h \
    $$PWD/include/utils/InPlaceOperator.h \
    $$PWD/include/utils/matlab.h

SOURCES += \
    $$PWD/src/matrix/SparseMatrix.cpp \
    $$PWD/src/matrix/DenseMatrix.cpp \
    $$PWD/src/vector/SparseVector.cpp \
    $$PWD/src/vector/DenseVector.cpp \
    $$PWD/src/decomposition/EVD.cpp \
    $$PWD/src/decomposition/LU.cpp \
    $$PWD/src/decomposition/QR.cpp \
    $$PWD/src/decomposition/SVD.cpp \
    $$PWD/src/utils/Utility.cpp \
    $$PWD/src/utils/Printer.cpp \
    $$PWD/src/utils/ArrayOperator.cpp \
    $$PWD/src/utils/InPlaceOperator.cpp \
    $$PWD/src/utils/Matlab.cpp

INCLUDEPATH += $$PWD/include/matrix
INCLUDEPATH += $$PWD/include/vector
INCLUDEPATH += $$PWD/include/decomposition
INCLUDEPATH += $$PWD/include/utils

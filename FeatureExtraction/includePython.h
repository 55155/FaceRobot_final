//includePython.h

#include <Python.h>
                                           

bool execute_AIspeaker_py() {
    //const wchar_t* PYTHON_HOME = L"c:\\users\\user\\anaconda3\\envs\\softface";
    const wchar_t* PYTHON_HOME = L"C:\\Users\\shy80\\anaconda3\\envs\\env_km_aispeaker";
    const char* PYTHON_SCRIPT_NAME = "chatGPT";

    // Set Python home
    Py_SetPythonHome(PYTHON_HOME);

    // Initialize Python interpreter
    Py_Initialize();

    // Add Python script path to Python module search path
    PyRun_SimpleString("import sys");
    PyRun_SimpleString("sys.path.append(r'AIspeaker_JA')");

    // Import Python module
    PyObject* pModuleName = PyUnicode_FromString(PYTHON_SCRIPT_NAME);
    PyObject* pModule = PyImport_Import(pModuleName);
    Py_DECREF(pModuleName);
    if (pModule == NULL) {
        PyErr_Print();
        return 0;
    }

    // Get Python function
    const char* PYTHON_FUNCTION_NAME = "chatGPT";
    PyObject* pFunction = PyObject_GetAttrString(pModule, PYTHON_FUNCTION_NAME);
    if (pFunction == NULL) {
        Py_DECREF(pModule);
        PyErr_Print();
        return 0;
    }

    // Call Python function
    PyObject* pArgs = NULL;
    PyObject* pReturnValue = PyObject_CallObject(pFunction, pArgs);
    Py_DECREF(pFunction);
    if (pReturnValue == NULL) {
        Py_DECREF(pModule);
        PyErr_Print();
        return 0;
    }

    // Clean up
    Py_DECREF(pModule);
    Py_DECREF(pReturnValue);
    Py_Finalize();

    return 1;
}

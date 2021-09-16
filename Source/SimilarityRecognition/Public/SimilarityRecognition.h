/************************************************************************/
/* Author: YWT20                                                        */
/* Expected release year : 2020                                         */
/************************************************************************/

#pragma once

#include "Modules/ModuleManager.h"

class FSimilarityRecognitionModule : public IModuleInterface
{
public:

	/** IModuleInterface implementation */
	virtual void StartupModule() override;
	virtual void ShutdownModule() override;
};

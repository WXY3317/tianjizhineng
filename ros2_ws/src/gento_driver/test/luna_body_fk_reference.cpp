// Offline oracle: use the vendor's original coordinate loader and update code.
#include "ICBase.h"
#include <cstdio>
#include <cstdlib>

class Reference : public CICBase
{
public:
    using CICBase::FX_OnLoadCord;
    using CICBase::FX_OnLoadCalLink;
    using CICBase::FX_OnLoadCalInput;
    using CICBase::FX_SearchCord;
};

int main(int argc, char **argv)
{
    if (argc != 10) return 1;
    Reference model;
    if (!model.FX_OnLoadCord(argv[1]) || !model.FX_OnLoadCalLink(argv[2]) ||
        !model.FX_OnLoadCalInput(argv[3])) return 2;
    double input[22] = {0};
    for (int i = 0; i < 6; ++i) input[i] = std::atof(argv[i + 4]);
    if (!model.FX_OnUpdateCord(22, input)) return 3;
    pCordCell flange = model.FX_SearchCord((char *)"BD_FL");
    if (!flange) return 4;
    for (int i = 0; i < 4; ++i)
        for (int j = 0; j < 4; ++j)
            std::printf("%.17g ", flange->m_RunW[i][j]);
    return 0;
}

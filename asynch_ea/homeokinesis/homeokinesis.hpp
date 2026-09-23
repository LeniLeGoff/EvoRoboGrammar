#include "ea_rg/rg_genome.hpp"

using namespace ea_rg;

enum HKType{
    HK = 0,
    CPGRBFHK
};

class HKInd : public RoboGrammarInd{
public:
    using Ptr = std::shared_ptr<HKInd>;
    using ConstPtr = std::shared_ptr<const HKInd>;
    HKInd() : RoboGrammarInd(){
        _morph_genome = std::make_shared<RoboGrammarGenome>();
        _ctrl_genome = std::make_shared<apear::EmptyGenome>();
    }
    HKInd(const apear::misc::RandNum::Ptr& rn, const apear::settings::ParametersMapPtr &param) :
        RoboGrammarInd(rn,param){
        _morph_genome = std::make_shared<RoboGrammarGenome>(rn,param);
        _ctrl_genome = std::make_shared<apear::EmptyGenome>();
    }
    HKInd(const RoboGrammarGenome::Ptr &morph_gen,const apear::EmptyGenome::Ptr &ctrl_gen) :
        RoboGrammarInd(morph_gen,ctrl_gen){}
    HKInd(const HKInd &ind) : RoboGrammarInd(ind){}

    Individual::Ptr clone() override{
        return std::make_shared<HKInd>(*this);
    }

    void init() override;
    void set_rules(const std::vector<RoboGrammarGenome::rule_idx_t> &rule_seq);

private:
    void _create_controller() override;
};



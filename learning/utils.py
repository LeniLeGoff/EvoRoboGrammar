import pyrobotdesign as rd

def string_to_rule_pairs(rules):
    rule_pairs = []
    for rule in rules:
        indexes = rule.split(",")
        rule_pairs.append([int(indexes[0]),int(indexes[1])])
    return rule_pairs

def convert_rule_pairs_to_list(grammar_file,rules):
    grammar = {0:[],1:[]}
    graphs = rd.load_graphs(grammar_file)
    j = 0
    k = 0
    for i,graph in zip(range(len(graphs)),graphs):
        rule = rd.create_rule_from_graph(graph)
        if len(rule.lhs.nodes) < len(rule.rhs.nodes):
            grammar[0].append((i,j))
            j+=1
        else:
            grammar[1].append((i,k))
            k+=1
    rule_list = []
    for rule in rules:
        rule_list.append(grammar[rule[0]][rule[1]][0])
    return rule_list
    
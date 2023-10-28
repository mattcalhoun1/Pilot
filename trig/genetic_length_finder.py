from trig.trig import BasicTrigCalc
import uuid
import numpy as np
import logging 
import time

# Performs a step-by-step search to find possible triangle lengths, given a far angle and far side
# the less confidence in a given side, the more it can be adjusted
class BaseTopLengthFinder:
    def __init__(self, far_angle, far_side, est_top, est_base, base_confidence = 0.8, top_confidence = 0.8):
        self.__far_angle = far_angle
        self.__far = far_side
        self.__est_top = est_top
        self.__est_base = est_base
        self.__trig_calc = BasicTrigCalc()

        # if we have less confidence in one of the variables, we are allowed to adjust it more
        self.__max_base_adjustment = ((1 - base_confidence) * 2)
        self.__max_top_adjustment = ((1 - top_confidence) * 2)

    def find_lengths (self, max_num_solutions = 5, target_accuracy = 0.01, allowed_time = None):
        #logging.getLogger(__name__).info(f"Given Far Side: {self.__far}, Far Angle: {self.__far_angle}, Solving For Base ~{self.__est_base}, top ~{self.__est_top}")

        optimizer = SideLengthOptimizer(
            far_angle=self.__far_angle,
            far_side=self.__far,
            population_size=max_num_solutions*15, 
            num_elites = max_num_solutions, 
            generations = 500 if allowed_time is None else 999999999,
            est_base= self.__est_base, 
            est_top=self.__est_top, 
            max_base_adjustment=self.__max_base_adjustment, 
            max_top_adjustment=self.__max_top_adjustment, 
            target_accuracy=target_accuracy)
        solutions = optimizer.select_side_lengths(max_solutions=max_num_solutions, allowed_time=allowed_time)

        # return the proposed solutions, along with how close they were able to get to the far angle
        # we have to test each angle, because if the search was stopped early, it may contain invalid solutions
        proposed = []
        for s in solutions:
            proposed_base, proposed_top = s.get_proposed_solution()
            base_angle = None
            max_bumps = 500
            curr_bump = 0
            adjusted_base_side = proposed_base
            adjusted_top_side = proposed_top
            while base_angle is None and curr_bump < max_bumps:
                curr_bump += 1
                try:
                    base_angle = self.__trig_calc.calc_base_angle(
                        far_side=self.__far, 
                        base_side=adjusted_base_side, 
                        top_side=adjusted_top_side
                    )
                    diff = self.__far_angle - self.__trig_calc.calc_far_angle(self.__far, adjusted_base_side, adjusted_top_side)
                except Exception:
                    # our estimated distances may not add up to a 180 degree agle,
                    # bump them until they do
                    adjusted_base_side += proposed_base * .01
                    adjusted_top_side += proposed_top * .01
            if base_angle is not None:
                proposed.append((adjusted_base_side, adjusted_top_side, diff))

            #logging.getLogger(__name__).info(f"Proposed - base side: {adjusted_base_side}, top side: {adjusted_top_side}")

        return proposed

# assigns a fitness metric to the proposed solution
class LengthFitnessTester:
    def __init__(self, far_angle, far_side):
        self.__far_angle = far_angle
        self.__far_side = far_side
        self.__trig_calc = BasicTrigCalc()

    def get_fitness_level (self, chromosome):
        # calculate the triangle, see how far it is from the desired angle
        score = 0.0
        try:
            proposed_base, proposed_top = chromosome.get_proposed_solution()
            #logging.getLogger(__name__).info(f"Testing base {proposed_base}, {proposed_top}")
            base_angle = self.__trig_calc.calc_base_angle(
                far_side=self.__far_side, 
                base_side=proposed_base, 
                top_side=proposed_top
            )

            # now given the all sides, see what it thinks the far angle is. However different that is becomes our difference


            # this could be a solution
            # See how far off we are on the angle, and score it
            score = 1.0 / abs(self.__far_angle - self.__trig_calc.calc_far_angle(self.__far_side, proposed_base, proposed_top))
            #logging.getLogger(__name__).info(f"Score: {score}")
        except:
            # this is not a valid triangle, return bad fitness score
            pass
        #logging.getLogger(__name__).info(f"Score: {score}")
        return score
    

class SideLengthOptimizer:
    def __init__(self, far_angle, far_side, population_size, num_elites, generations, est_base, est_top, max_base_adjustment=0.5, max_top_adjustment=0.5, target_accuracy=None):
        self.__far = far_side
        self.__far_angle = far_angle
        self.__tester = LengthFitnessTester(far_angle=self.__far_angle, far_side=self.__far)
        self.__population_size = population_size
        self.__max_generations = generations
        self.__num_elites = num_elites
        self.__max_base_adjustment = max_base_adjustment
        self.__max_top_adjustment = max_top_adjustment
        self.__est_base = est_base
        self.__est_top = est_top
        self.__trig_calc = BasicTrigCalc()
        self.__target_accuracy = target_accuracy

    def generate_random_chromosomes (self, num_chromosomes, add_starting_estimates = False):
        gene_key = {
            'base':0,
            'top':1,
            'far':2, # not adjustable
            'far_angle':3 # not adjustable
        }

        # max range of base and top
        gene_ranges = [
            [self.__est_base - (self.__est_base * self.__max_base_adjustment),self.__est_base + (self.__est_base * self.__max_base_adjustment)], # base range
            [self.__est_top - (self.__est_top * self.__max_top_adjustment),self.__est_top + (self.__est_top * self.__max_top_adjustment)], # top range
        ]
        c = []

        for i in range(num_chromosomes):
            init_genes = []
            for gr in gene_ranges:
                init_genes.append(np.random.uniform(gr[0],gr[1]))
            c.append(Chromosome(
                gene_key=gene_key, 
                gene_ranges=gene_ranges,
                genes=np.array(init_genes + [self.__far, self.__far_angle]),
                calc=self.__trig_calc))
            
        if add_starting_estimates:
            c.append(Chromosome(
                gene_key=gene_key, 
                gene_ranges=gene_ranges,
                genes=np.array([self.__est_base, self.__est_top, self.__far, self.__far_angle]),
                calc=self.__trig_calc))
        
        # randomly adjust the top or base so each one fits
        for curr_c in c:
            makefit_gene = np.random.randint(0,2)
            curr_c.make_fit(makefit_gene)


        return c

    def select_side_lengths (self, max_solutions = 1, allowed_time = None):
        start_time = time.time()
        #logging.getLogger(__name__).info("Finding side lengths")
        # use genetic algo to select best side lengths

        # initialize some chromosomes
        pop = self.generate_random_chromosomes(self.__population_size, add_starting_estimates=True)

        reproducer = Reproducer()

        best_chromosome = None
        best_fitness = 0
        generations_without_best = 0

        stagnation_threshold = 50
        last_ranked_pop = []

        # while stop condition not met
        for generation_count in range(self.__max_generations+1):
            #print (f'generation: {generation_count}, pop size: {len(pop)}')
            fitness_map = {}

            # test the fitness of each chromosome
            for c in pop:
                fitness_map[c.get_id()] = self.__tester.get_fitness_level(c)

            # rank each chromosome by fitness
            ranked_chromosome_ids = [k for k, v in sorted(fitness_map.items(), key=lambda item: item[1], reverse=True)]

            # reorder the population by fitness map
            # if the desired number have achieved the target, stop searching
            ranked_pop = []
            target_count = 0
            for c_id in ranked_chromosome_ids:
                ranked_pop.append([p for p in pop if p.get_id() == c_id][0])
                # 1/angle diff is the score, so 1/the score will give us the diff
                if fitness_map[c_id] > 0 and 1/fitness_map[c_id] <= self.__target_accuracy * self.__far_angle:
                    target_count += 1
            if target_count >= max_solutions:
                #logging.getLogger(__name__).info(f"Stopping early, desired accuracy was {self.__target_accuracy}")
                return ranked_pop[0:max_solutions]

            # if too much time has passed, return whatever we have
            #logging.getLogger(__name__).info(f"elapsed: {time.time() - start_time} - allowed: {allowed_time}")
            if allowed_time is not None and (time.time() - start_time) > allowed_time:
                if target_count >= max_solutions:
                    #logging.getLogger(__name__).info(f"Stopping early, desired accuracy was {self.__target_accuracy}")
                    return ranked_pop[0:max_solutions]
                else:
                    return ranked_pop[0:len(ranked_pop)]

            

            last_ranked_pop = ranked_pop

            # compare the best chromosome with our best so far
            best_fitness_this_generation = fitness_map[ranked_pop[0].get_id()]
            if best_fitness_this_generation > best_fitness:
                best_fitness = best_fitness_this_generation
                best_chromosome = ranked_pop[0]                
                generations_without_best = 0
                reproducer.reset_mutation_rate()
            else:
                generations_without_best += 1

                # increase mutations every .1*stag threshold generations, if nothing is happening
                if generations_without_best > stagnation_threshold and generations_without_best % int(.1*stagnation_threshold) == 0:
                    reproducer.increase_mutations()
                else:
                    reproducer.reset_mutation_rate()

            probabilities = np.linspace(.9,.05,len(ranked_chromosome_ids))

            # scale the probabilities so they add up to one
            probabilities = probabilities / np.sum(probabilities)

            # select the reproducers using a weighted choice, without replacement
            # the weights are basically ordered by the rank
            reproducers = np.random.choice(
                len(ranked_chromosome_ids), 
                size=self.__population_size, 
                p=probabilities, 
                replace=len(ranked_chromosome_ids) < self.__population_size # dont' want to reproduce with self, unless no choice
            )

            # no more than 1/3 can reproduce
            if len(reproducers) > int(self.__population_size / 3):
                reproducers = reproducers[0:int(self.__population_size / 3)]

            new_pop = []
            
            # keep elites (if we had some and if we're using elites)
            for e_count in range(self.__num_elites):
                if len(ranked_pop) > e_count:
                    new_pop.append(ranked_pop[e_count])

            # reproduce
            for p in range(0, len(reproducers), 2):
                # random number of offspring per couple, between 1 and 2 offspring
                for offspring_count in range(0, np.random.randint(1,4)):
                    if len(new_pop) < self.__population_size and len(reproducers) > p+1:
                        new_pop.append(reproducer.get_offspring([ranked_pop[reproducers[p]], ranked_pop[reproducers[p+1]]]))

            # remove clones
            unique_pop = []
            used_pairs = []
            for p in new_pop:
                s = p.get_summary()
                if s not in used_pairs:
                    unique_pop.append(p)
                    used_pairs.append(s)
            new_pop = unique_pop

            # fill in the rest of the population with new random chromosomes
            if len(new_pop) < self.__population_size:
                fresh_pop = self.generate_random_chromosomes(self.__population_size - len(new_pop))
                new_pop = new_pop + fresh_pop



            # throw out the old generation if we have enough. otherwise, append
            #if len(new_pop) < int(.8*self.__population_size):
            #    pop = pop + new_pop
            #else:
            pop = new_pop

        if max_solutions <= len(last_ranked_pop):
            return last_ranked_pop[0:max_solutions]

        return last_ranked_pop[0:len(last_ranked_pop)]

class Chromosome:
    def __init__(self, gene_key, gene_ranges, genes, calc):
        self.__gene_key = gene_key # list of field names that correspond with the given gene array
        self.__gene_ranges = gene_ranges
        self.__genes = genes
        self.__id = uuid.uuid4()
        self.__trig_calc = calc

    # adjusts the specified gene to make an overall fit
    # this helps arrive at a solution faster, likely
    # at the expense of less diversity in the solutions
    def make_fit (self, adjust_gene):
        step_size = 0.005
        step_factors = range(0,100,2) # we will move in either side, up to 50% to make a fit

        made_fit = False
        if adjust_gene == self.__gene_key['top']:
            # bump the top side around as necessary to arrive at a possible fit for this selected base
            for factor in step_factors:
                try:
                    #logging.getLogger(__name__).info(f"Trying top size: {self.__genes[self.__gene_key['top']] + (step_size * factor * self.__genes[self.__gene_key['top']])}")
                    trial = self.__trig_calc.calc_base_side (
                        far_angle = self.__genes[self.__gene_key['far_angle']],
                        far_side = self.__genes[self.__gene_key['far']],
                        top_side = self.__genes[self.__gene_key['top']] + (step_size * factor * self.__genes[self.__gene_key['top']]))
                    self.__genes[self.__gene_key['top']] = self.__genes[self.__gene_key['top']] + (step_size * factor * self.__genes[self.__gene_key['top']])
                    made_fit = True
                    break
                except:
                    try:
                        if factor > 0:
                            #logging.getLogger(__name__).info(f"Trying top size: {self.__genes[self.__gene_key['top']] - (step_size * factor  * self.__genes[self.__gene_key['top']])}")
                            trial = self.__trig_calc.calc_base_side (
                                far_angle = self.__genes[self.__gene_key['far_angle']],
                                far_side = self.__genes[self.__gene_key['far']],
                                top_side = self.__genes[self.__gene_key['top']] - (step_size * factor  * self.__genes[self.__gene_key['top']]))
                            self.__genes[self.__gene_key['top']] = self.__genes[self.__gene_key['top']] - (step_size * factor  * self.__genes[self.__gene_key['top']])
                            made_fit = True
                            break
                    except:
                        pass # will bounce around
            #if not made_fit:
            #    logging.getLogger(__name__).info(f"Unable to make fit far angle: {self.__genes[self.__gene_key['far_angle']]}, far side: {self.__genes[self.__gene_key['far']]}, base side: {self.__genes[self.__gene_key['base']]}")
        elif adjust_gene == self.__gene_key['base']:
            # set the base side to whatever is necessary to complete the triangle
            for factor in step_factors:
                try:
                    trial = self.__trig_calc.calc_top_side (
                        far_angle = self.__genes[self.__gene_key['far_angle']],
                        far_side = self.__genes[self.__gene_key['far']],
                        base_side = self.__genes[self.__gene_key['base']] + (step_size * factor  * self.__genes[self.__gene_key['base']]))
                    self.__genes[self.__gene_key['base']] = self.__genes[self.__gene_key['base']] + (step_size * factor  * self.__genes[self.__gene_key['base']])
                    made_fit = True
                    break
                except:
                    try:
                        if factor > 0:
                            trial = self.__trig_calc.calc_top_side (
                                far_angle = self.__genes[self.__gene_key['far_angle']],
                                far_side = self.__genes[self.__gene_key['far']],
                                base_side = self.__genes[self.__gene_key['top']] - (step_size * factor  * self.__genes[self.__gene_key['base']]))
                            self.__genes[self.__gene_key['base']] = self.__genes[self.__gene_key['base']] - (step_size * factor  * self.__genes[self.__gene_key['base']])
                            made_fit = True
                            break
                    except:
                        pass # will bounce around
            #if not made_fit:
            #    logging.getLogger(__name__).info(f"Unable to make fit far angle: {self.__genes[self.__gene_key['far_angle']]}, far side: {self.__genes[self.__gene_key['far']]}, base side: {self.__genes[self.__gene_key['base']]}")



        else:
            logging.getLogger(__name__).warning(f"Code error, as chromosome was told to make fit gene: {adjust_gene}")

    def get_gene_key (self):
        return self.__gene_key

    def get_gene_count (self):
        return len(self.__genes)

    def get_editable_gene_count (self):
        return len(self.__genes) - 2

    def get_genes (self):
        return self.__genes

    def take_genes (self, new_genes, parent):
        for i in new_genes:
            self.__genes[i] = parent.get_genes()[i]
            to_adjust = ((i+1) % 2)

            #logging.getLogger(__name__).info(f"Taking gene: {i}, adjusting as necessary gene: {to_adjust}")

            # make sure still fits by adjusting the existing gene
            self.make_fit(to_adjust)

    def mutate_genes (self, genes):
        # somehow mutate the given gene to be within valid ranges
        for g in genes:
            try:
                min_allowed = self.__gene_ranges[g][0]
                max_allowed = self.__gene_ranges[g][1]

                if isinstance(min_allowed, int):
                    self.__genes[g] = np.random.randint(min_allowed, max_allowed+1)
                elif isinstance(min_allowed, float):
                    self.__genes[g] = np.random.uniform(min_allowed, max_allowed)
            except:
                # the top end of genes are read only, so can't be mutatd
                pass

    def get_proposed_solution (self):
        return self.__genes[self.__gene_key['base']], self.__genes[self.__gene_key['top']]

    def get_summary (self):
        return ', '.join(f'{k}:{self.__genes[self.__gene_key[k]]}' for k in self.__gene_key)

    def get_id (self):
        return self.__id

    def clone (self):
        return Chromosome(self.__gene_key, self.__gene_ranges, np.copy(self.__genes), self.__trig_calc)

class Reproducer:
    def __init__(self):
        self.__normal_mutation_rate = .33
        self.__normal_max_gene_mutations = 1
        self.__mutation_rate = self.__normal_mutation_rate
        self.__max_gene_mutations = self.__normal_max_gene_mutations

    def reset_mutation_rate (self):
        self.__mutation_rate = self.__normal_mutation_rate
        self.__max_gene_mutations = self.__normal_max_gene_mutations

    def increase_mutations (self):
        self.__mutation_rate = 2*self.__normal_mutation_rate
        self.__max_gene_mutations = 2.0*self.__normal_max_gene_mutations
        #print ('increasing mutations')

    def get_offspring(self, parents):
        # parents are shuffled so their order wont matter
        p1_index = np.random.randint(0,2)
        p1 = parents[p1_index]
        p2 = parents[1 - p1_index]
        
        # select a random number 'r' between 1 and 3
        gene_selector = np.random.randint(1, 2) # every nth gene
        beginning_gene = np.random.randint(0, 2) # must be 0 or 1

        # teh offspring gets every rth gene value from parent 1
        offspring = p1.clone()
        offspring.take_genes (np.arange(beginning_gene, offspring.get_editable_gene_count(), gene_selector), p2)

        # possibly introduce mutation
        if np.random.randint(0, 101) < (self.__mutation_rate*100):
            genes_to_flip = np.random.choice(
                len(offspring.get_genes()), 
                size=np.random.randint(1, self.__max_gene_mutations+1),
                replace=False)
            offspring.mutate_genes (genes_to_flip)

        return offspring
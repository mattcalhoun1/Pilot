from multiprocessing import Pool as ThreadPool
from multiprocessing.context import TimeoutError
import atexit
import logging

# this class ensures that all estimators share a single
# thread pool, if using multithreading. This pool stays
# alive for the life of the process
class PositionThreadManager:
    __thread_pool = None

    def get_thread_pool ():
        # the pool size of 3 seems to work best, based on unit tests, on the pi cm4 (4 cores).
        # this makes sense, because we are currently limiting the number of landmarks taken into account
        # to a max of 3, so 3 sets of calculations need to be done.
        pool_size = 3
        if PositionThreadManager.__thread_pool is None:
            atexit.register(PositionThreadManager.cleanup)
            logging.getLogger(__name__).warning(f"Position thread pool initializing with {pool_size} workers. This should only happen once.")
            PositionThreadManager.__thread_pool = ThreadPool(pool_size)
        
        return PositionThreadManager.__thread_pool
    
    def cleanup ():
        if PositionThreadManager.__thread_pool is not None:
            logging.getLogger(__name__).warning("Cleaning up position thread pool")
            PositionThreadManager.__thread_pool.close()
            PositionThreadManager.__thread_pool.terminate()
            PositionThreadManager.__thread_pool = None
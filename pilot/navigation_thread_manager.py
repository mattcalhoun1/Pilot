from multiprocessing import Pool as ThreadPool
from multiprocessing.context import TimeoutError
import atexit
import logging

# manages pool of threads available for navigation
class NavigationThreadManager:
    __thread_pool = None

    def get_thread_pool ():
        # this should match the number of cameras, since pilot-related multi-threading
        # is mainly to allow concurrent processing involving the cameras
        pool_size = 2
        if NavigationThreadManager.__thread_pool is None:
            atexit.register(NavigationThreadManager.cleanup)
            logging.getLogger(__name__).warning(f"Pilot thread pool initializing with {pool_size} workers. This should only happen once.")
            NavigationThreadManager.__thread_pool = ThreadPool(pool_size)
        
        return NavigationThreadManager.__thread_pool
    
    def cleanup ():
        if NavigationThreadManager.__thread_pool is not None:
            logging.getLogger(__name__).warning("Cleaning up pilot thread pool")
            NavigationThreadManager.__thread_pool.close()
            NavigationThreadManager.__thread_pool.terminate()
            NavigationThreadManager.__thread_pool = None
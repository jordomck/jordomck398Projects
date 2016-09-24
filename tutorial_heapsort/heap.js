
// create empty object 
minheaper = { 
}; 

function parentOf(num){
	return Math.floor(num / 2);
}

function childOne(num) {
	return num * 2;
}

function childTwo(num) {
	return (num * 2) + 1;
}

function swap(array, idx1, idx2){
	var temp = array[idx1];
	array[idx1] = array[idx2];
	array[idx2] = temp;
}

// define insert function for min binary heap
function minheap_insert(heap, new_element) {
	heap.push(new_element);
	var idx = heap.length - 1;
	while(heap[idx] < heap[parentOf(idx)]){
		swap(heap, idx, parentOf(idx));
		idx = parentOf(idx);
	}
    // STENCIL: implement your min binary heap insert operation
}

// assign insert function within minheaper object
minheaper.insert = minheap_insert;
/* Note: because the minheap_insert function is an object, we can assign 
      a reference to the function within the minheap object, which can be called
      as minheap.insert
*/

// define extract function for min binary heap
function minheap_extract(heap) {
	if(heap.length == 0) return;
	var valuePulled = heap[0];
	heap[0] = heap.pop();
	var idx = 0;
	while(childOne(idx) <= heap.length && childTwo(idx) <= heap.length && heap[idx] > heap[childOne(idx)] && heap[idx] > heap[childTwo(idx)]){
		if(heap[idx] > heap[childOne(idx)]){
			swap(heap, idx, childOne(idx));
			idx = childOne(idx);
		}
		else if(heap[idx] > heap[childTwo(idx)]){
			swap(heap, idx, childTwo(idx));
			idx = childTwo(idx);
		}
	}
	return valuePulled;
    // STENCIL: implement your min binary heap extract operation
}

// assign extract function within minheaper object
minheaper.extract = minheap_extract;
    // STENCIL: ensure extract method is within minheaper object






